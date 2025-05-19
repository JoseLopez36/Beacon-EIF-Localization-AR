#include "gz_uwb_beacon_plugin/gz_uwb_beacon_plugin.h"

namespace gz
{

	GzUwbBeaconPlugin::GzUwbBeaconPlugin()
	{
		// Inicializar el periodo de actualización
		update_period_ = std::chrono::nanoseconds(0);
		// Initialize ROS2 clock
		ros_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);
		// Inicializar el generador de números aleatorios
		std::random_device rd;
		random_generator_ = std::default_random_engine(rd());
	}

	void GzUwbBeaconPlugin::Configure(const sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
		sim::EntityComponentManager& _ecm, sim::EventManager& _eventMgr)
	{
		// Inicializar nodo ROS2
		if (!rclcpp::ok())
			rclcpp::init(0, nullptr);

		node_ = std::make_shared<rclcpp::Node>("gz_uwb_beacon_plugin");

		// Obtener entidades del mundo y modelo
		world_entity_ = _ecm.EntityByComponents(sim::components::World());
		model_entity_ = _entity;

		// Verificar que las entidades son válidas
		if (world_entity_ == sim::kNullEntity) {
			RCLCPP_WARN(node_->get_logger(), "UWB-Beacon-Plugin: World entity not found!");
		}

		if (model_entity_ == sim::kNullEntity) {
			RCLCPP_FATAL(node_->get_logger(), "UWB-Beacon-Plugin: Model entity is null!");
			return;
		}

		// Obtener parámetros del plugin
		double update_rate = _sdf->Get<double>("update_rate");
		update_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / update_rate));
		beacon_prefix_ = "uwb_beacon_";
		tag_id_ = 0;
		nlos_soft_wall_width_ = 0.25;
		tag_z_offset_ = 0;
		max_db_distance_ = 14;
		step_db_distance_ = 0.1;
		use_parent_as_reference_ = true;

		if (_sdf->HasElement("beacon_prefix"))
		{
			beacon_prefix_ = _sdf->Get<std::string>("beacon_prefix");
		}

		if (_sdf->HasElement("tag_id"))
		{
			tag_id_ = _sdf->Get<int>("tag_id");
		}

		if (_sdf->HasElement("nlos_soft_wall_width"))
		{
			nlos_soft_wall_width_ = _sdf->Get<double>("nlosSoftWallWidth");
		}

		if (_sdf->HasElement("tag_z_offset"))
		{
			tag_z_offset_ = _sdf->Get<double>("tag_z_offset");
		}

		RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: Plugin is running. Tag ID: %d", tag_id_);
		RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: All parameters loaded");

		last_update_time_ = std::chrono::steady_clock::time_point();

		// Inicializar publicadores de medidas
		measurement_pubs_.clear();

		// Crear publicador de datos de marcadores
		std::string markers_topic = "/uwb_beacon/markers";
		markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic, 100);
		RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: Anchors Position Publishing in %s", markers_topic.c_str());
	}

	void GzUwbBeaconPlugin::Reset(const sim::UpdateInfo& _info, sim::EntityComponentManager& _ecm)
	{
		RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: Resetting Plugin");
		last_update_time_ = std::chrono::steady_clock::now();
	}

	void GzUwbBeaconPlugin::PreUpdate(const sim::UpdateInfo& _info, sim::EntityComponentManager& _ecm)
	{
		// Verificar si es tiempo de actualizar
		auto current_time = std::chrono::steady_clock::now();
		auto elapsed = current_time - last_update_time_;

		if (elapsed < update_period_)
			return;

		last_update_time_ = current_time;

		// Lógica de actualización
		// Obtener la posición del tag
		math::Pose3d tag_pose;
		if (!use_parent_as_reference_)
		{
			auto tag_pose_comp = _ecm.Component<sim::components::Pose>(tag_link_entity_);
			// Check if tag_pose_comp is valid before accessing it
			if (!tag_pose_comp) {
				RCLCPP_ERROR(node_->get_logger(), "UWB-Beacon-Plugin: Tag pose component is null, cannot continue");
				return;
			}
			tag_pose = tag_pose_comp->Data();
		}
		else
		{
			// Obtener pose del modelo
			auto model_pose_gz_comp = _ecm.Component<sim::components::Pose>(model_entity_);
			if (model_pose_gz_comp)
				tag_pose = model_pose_gz_comp->Data();
			else
				RCLCPP_FATAL(node_->get_logger(), "UWB-Beacon-Plugin: Error, model pose not found");
		}

		// Ajustar la posición del tag en el eje Z
		math::Vector3d pos_corrected_Z(tag_pose.Pos().X(), tag_pose.Pos().Y(), tag_pose.Pos().Z() + tag_z_offset_);
		tag_pose.Set(pos_corrected_Z, tag_pose.Rot());
		math::Vector3d current_tag_pose(tag_pose.Pos());

		// Obtener los ángulos de Euler del tag
		tf2::Quaternion q(tag_pose.Rot().X(), tag_pose.Rot().Y(), tag_pose.Rot().Z(), tag_pose.Rot().W());
		tf2::Matrix3x3 m(q);
		double roll, pitch, current_yaw;
		m.getRPY(roll, pitch, current_yaw);

		// Calcular el barrido de ángulos de prueba
		double start_angle = current_yaw;
		double current_angle = 0;
		double arc = 3.0 * M_PI / 2.0;
		int num_angles_to_test_by_side = 30;
		double increment_angle = arc / num_angles_to_test_by_side;
		int total_number_angles_to_test = 1 + 2 * num_angles_to_test_by_side;
		double angles_to_test[total_number_angles_to_test];
		angles_to_test[0] = start_angle;
		for (int i = 1; i < total_number_angles_to_test; ++i)
		{
			double angle_to_test;
			if (i % 2 == 0)
			{
				angle_to_test = start_angle - (i / 2) * increment_angle;
			}
			else
			{
				angle_to_test = start_angle + (i - (i - 1) / 2) * increment_angle;
			}
			angles_to_test[i] = angle_to_test;
		}

		// Crear los arrays de marcadores
		visualization_msgs::msg::MarkerArray marker_array;
		visualization_msgs::msg::MarkerArray interferences_array;

		// Iterar sobre todos los modelos del mundo para encontrar las balizas
		auto models = _ecm.EntitiesByComponents(sim::components::Model());

		for (const auto& model_entity : models)
		{
			auto name_comp = _ecm.Component<sim::components::Name>(model_entity);
			if (name_comp && name_comp->Data().find(beacon_prefix_) == 0)
			{
				// Baliza encontrada. Obtener su ID y pose
				std::string bid_str = name_comp->Data().substr(beacon_prefix_.length());
				int bid = std::stoi(bid_str);

				auto beacon_pose_comp = _ecm.Component<sim::components::Pose>(model_entity);
				math::Pose3d beacon_pose = beacon_pose_comp ? beacon_pose_comp->Data() : math::Pose3d();

				// Inicializar variables de la baliza
				LineOfSight los_type = LOS;
				double distance = tag_pose.Pos().Distance(beacon_pose.Pos());
				double distance_after_rebounds = 0;

				// Comprobar si hay un obstáculo entre la baliza y el tag
				double distance_to_obstacle_from_tag = 0.0;
				std::string obstacle_name = "";
				obstacle_name = getIntersection(_ecm, tag_pose.Pos(), beacon_pose.Pos(), distance_to_obstacle_from_tag);

				if (obstacle_name.compare("") == 0)
				{
					// No hay obstáculo entre la baliza y el tag, se usa el modelo Line-Of-Sight
					los_type = LOS;
					distance_after_rebounds = distance;
					// RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: LOS. No obstacles between tag and anchor");
				}
				else
				{
					// Hay un obstáculo entre la baliza y el tag, se usa el modelo Non-Line-Of-Sight
					// Se usa un rayo adicional para medir la distancia desde la baliza al tag, 
					// para conocer el ancho de las paredes
					double distance_to_obstacle_from_anchor = 0.0;
					std::string other_obstacle_name = "";
					other_obstacle_name = getIntersection(_ecm, beacon_pose.Pos(), tag_pose.Pos(), distance_to_obstacle_from_anchor);

					double wall_width = distance - distance_to_obstacle_from_tag - distance_to_obstacle_from_anchor;
					if (wall_width <= nlos_soft_wall_width_ && obstacle_name.compare(other_obstacle_name) == 0)
					{
						// Se usa el modelo Non-Line-Of-Sight - Soft
						los_type = NLOS_S;
						distance_after_rebounds = distance;
						// RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: NLOS_S. Wall width: %f", wall_width);
					}
					else
					{
						// Buscamos un rebote para llegar a la baliza desde el tag
						bool end = false;
						double max_distance = 30.0;
						double distance_to_rebound = 0.0;
						double distance_to_final_obstacle = 0.0;
						double distance_nlos_hard = 0.0;
						double step_floor = 1.0;
						double start_floor_distance_check = 2.0;
						int num_steps_floor = 6;
						std::string final_obstacle_name;
						int index_ray = 0;
						bool found_nlos_h = false;
						int current_floor_distance = 0;
						while (!end)
						{
							current_angle = angles_to_test[index_ray];

							double x = current_tag_pose.X() + max_distance * cos(current_angle);
							double y = current_tag_pose.Y() + max_distance * sin(current_angle);
							double z = current_tag_pose.Z();

							if (current_floor_distance > 0)
							{
								double tan_angle_floor =
									(start_floor_distance_check + step_floor * (current_floor_distance - 1)) / current_tag_pose.Z();
								double angle_floor = atan(tan_angle_floor);

								double h = sin(angle_floor) * max_distance;

								double horizontal_distance = sqrt(max_distance * max_distance - h * h);

								x = current_tag_pose.X() + horizontal_distance * cos(current_angle);
								y = current_tag_pose.Y() + horizontal_distance * sin(current_angle);
								z = -1.0 * (h - current_tag_pose.Z());

							}

							math::Vector3d ray_point(x, y, z);

							obstacle_name = getIntersection(_ecm, current_tag_pose, ray_point, distance_to_rebound);

							if (obstacle_name.compare("") != 0)
							{
								math::Vector3d collision_point(
									current_tag_pose.X() + distance_to_rebound * cos(current_angle),
									current_tag_pose.Y() + distance_to_rebound * sin(current_angle),
									current_tag_pose.Z());

								if (current_floor_distance > 0)
								{
									collision_point.Set(
										current_tag_pose.X() + distance_to_rebound * cos(current_angle),
										current_tag_pose.Y() + distance_to_rebound * sin(current_angle),
										0.0);
								}

								final_obstacle_name = getIntersection(_ecm, collision_point, beacon_pose.Pos(), distance_to_final_obstacle);

								if (final_obstacle_name.compare("") == 0)
								{
									// Llegamos a la baliza después de un rebote
									distance_to_final_obstacle = beacon_pose.Pos().Distance(collision_point);
									if (distance_to_rebound + distance_to_final_obstacle <= max_db_distance_)
									{
										found_nlos_h = true;

										// Buscamos el rebote más corto
										if (distance_nlos_hard < 0.1)
										{
											distance_nlos_hard = distance_to_rebound + distance_to_final_obstacle;
										}
										else if (distance_nlos_hard > distance_to_rebound + distance_to_final_obstacle)
										{
											distance_nlos_hard = distance_to_rebound + distance_to_final_obstacle;
										}
									}
								}
							}

							if (index_ray < total_number_angles_to_test - 1)
							{
								index_ray += 1;
							}
							else
							{
								if (current_floor_distance < num_steps_floor)
								{
									current_floor_distance += 1;
									index_ray = 0;

								}
								else
								{
									end = true;
								}

							}
						}

						if (found_nlos_h)
						{
							// Usamos el modelo Non-Line-Of-Sight - Hard con distancia = distance_nlos_hard
							los_type = NLOS_H;
							distance_after_rebounds = distance_nlos_hard;
							// RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: NLOS_H. Wall width: %f", wall_width);
						}
						else
						{
							// No podemos llegar a la baliza, no hay ranging
							los_type = NLOS;
							// RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: NLOS. No way to reach the anchor");
						}
					}
				}

				if ((los_type == LOS || los_type == NLOS_S) && distance_after_rebounds > max_db_distance_)
				{
					los_type = NLOS;
				}

				if (los_type == NLOS_H && distance_after_rebounds > max_db_distance_)
				{
					los_type = NLOS;
				}

				if (los_type != NLOS)
				{
					int index_scenario = 0;
					if (los_type == NLOS_S)
					{
						index_scenario = 2;
					}
					else if (los_type == NLOS_H)
					{
						index_scenario = 1;
					}

					// Add bounds checking for array indices
					int index_ranging_offset = (int)round(distance_after_rebounds / step_db_distance_);
					if (index_ranging_offset < 0 || index_ranging_offset >= 141) {
						RCLCPP_ERROR(node_->get_logger(), "UWB-Beacon-Plugin: index_ranging_offset out of bounds: %d", index_ranging_offset);
						continue;
					}

					double distance_after_rebounds_with_offset = distance_after_rebounds;
					if (los_type == LOS)
					{
						distance_after_rebounds_with_offset =
							distance_after_rebounds + ranging_offset_[index_ranging_offset][0] / 1000.0;
					}
					else if (los_type == NLOS_S)
					{
						distance_after_rebounds_with_offset =
							distance_after_rebounds + ranging_offset_[index_ranging_offset][1] / 1000.0;
					}

					int index_ranging = (int)round(distance_after_rebounds_with_offset / step_db_distance_);
					if (index_ranging < 0 || index_ranging >= 141) {
						RCLCPP_ERROR(node_->get_logger(), "UWB-Beacon-Plugin: index_ranging out of bounds: %d", index_ranging);
						continue;
					}

					if (index_scenario < 0 || index_scenario >= 3) {
						RCLCPP_ERROR(node_->get_logger(), "UWB-Beacon-Plugin: index_scenario out of bounds: %d", index_scenario);
						continue;
					}

					std::normal_distribution<double> distribution_ranging(
						distance_after_rebounds_with_offset * 1000, ranging_std_[index_ranging][index_scenario]);
					std::normal_distribution<double> distribution_rss(
						rss_mean_[index_ranging][index_scenario], rss_std_[index_ranging][index_scenario]);

					double ranging_value = distribution_ranging(random_generator_);
					double power_value = distribution_rss(random_generator_);

					if (power_value < min_power_[index_scenario])
					{
						los_type = NLOS;
					}


					if (los_type != NLOS)
					{
						gz_uwb_beacon_msgs::msg::Measurement measurement_msg;
						measurement_msg.header.stamp = ros_clock_.now();
						measurement_msg.beacon_id = bid;
						measurement_msg.tag_id = tag_id_;
						measurement_msg.distance = ranging_value;
						measurement_msg.rss = power_value;
						measurement_msg.error_estimation = 0.00393973;
						if (measurement_pubs_.find(bid) != measurement_pubs_.end())
						{
							// Si el publicador para la baliza existe
							measurement_pubs_[bid]->publish(measurement_msg);
						}
						else
						{
							// Si no existe, crear un nuevo publicador
							std::string measurement_topic = "/uwb_beacon/" + name_comp->Data() + "/measurement";
							measurement_pubs_[bid] = node_->create_publisher<gz_uwb_beacon_msgs::msg::Measurement>(measurement_topic, 100);
							RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: New beacon found. Ranging Publishing in %s", measurement_topic.c_str());
						}
					}
				}

				// Marcador para el cuerpo de la baliza (cilindro central)
				visualization_msgs::msg::Marker body_marker;
				body_marker.header.frame_id = "map";
				body_marker.header.stamp = ros_clock_.now();
				body_marker.id = bid * 4;
				body_marker.type = visualization_msgs::msg::Marker::CYLINDER;
				body_marker.action = visualization_msgs::msg::Marker::ADD;
				body_marker.pose.position.x = beacon_pose.Pos().X();
				body_marker.pose.position.y = beacon_pose.Pos().Y();
				body_marker.pose.position.z = beacon_pose.Pos().Z() - 0.25;
				body_marker.pose.orientation.x = beacon_pose.Rot().X();
				body_marker.pose.orientation.y = beacon_pose.Rot().Y();
				body_marker.pose.orientation.z = beacon_pose.Rot().Z();
				body_marker.pose.orientation.w = beacon_pose.Rot().W();
				body_marker.scale.x = 0.25;
				body_marker.scale.y = 0.25;
				body_marker.scale.z = 0.5;
				body_marker.color.a = 1.0;
				body_marker.color.r = 0.0;
				body_marker.color.g = 0.0;
				body_marker.color.b = 1.0;
				marker_array.markers.push_back(body_marker);

				// Marcador para la base de la baliza (cilindro más grueso)
				visualization_msgs::msg::Marker base_marker;
				base_marker.header = body_marker.header;
				base_marker.id = bid * 4 + 1;
				base_marker.type = visualization_msgs::msg::Marker::CYLINDER;
				base_marker.action = visualization_msgs::msg::Marker::ADD;
				base_marker.pose = body_marker.pose;
				base_marker.pose.position.z = beacon_pose.Pos().Z() - 0.5;
				base_marker.scale.x = 0.4;
				base_marker.scale.y = 0.4;
				base_marker.scale.z = 0.2;
				base_marker.color = body_marker.color;
				marker_array.markers.push_back(base_marker);

				// Marcador para la esfera superior (origen de la baliza)
				visualization_msgs::msg::Marker sphere_marker;
				sphere_marker.header = body_marker.header;
				sphere_marker.id = bid * 4 + 2;
				sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
				sphere_marker.action = visualization_msgs::msg::Marker::ADD;
				sphere_marker.pose = body_marker.pose;
				sphere_marker.pose.position.z = beacon_pose.Pos().Z();
				sphere_marker.scale.x = 0.4;
				sphere_marker.scale.y = 0.4;
				sphere_marker.scale.z = 0.4;
				sphere_marker.color = body_marker.color;
				marker_array.markers.push_back(sphere_marker);

				// Marcador para la línea entre la baliza y el vehículo (LOS)
				visualization_msgs::msg::Marker los_marker;
				los_marker.header = body_marker.header;
				los_marker.id = bid * 4 + 3;
				los_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
				los_marker.action = visualization_msgs::msg::Marker::ADD;
				los_marker.pose.orientation.w = 1.0;
				los_marker.scale.x = 0.03;  // Grosor de la línea
				los_marker.color.a = 1.0;

				// Añadir los puntos de la línea
				geometry_msgs::msg::Point p1, p2;
				p1.x = beacon_pose.Pos().X();
				p1.y = beacon_pose.Pos().Y();
				p1.z = beacon_pose.Pos().Z();
				p2.x = tag_pose.Pos().X();
				p2.y = tag_pose.Pos().Y();
				p2.z = tag_pose.Pos().Z();
				los_marker.points.push_back(p1);
				los_marker.points.push_back(p2);

				// Colores de las líneas de visión según el tipo de Line-Of-Sight
				if (los_type == LOS)
				{
					// Verde
					los_marker.color.r = 0.0;
					los_marker.color.g = 1.0;
					los_marker.color.b = 0.0;
				}
				else if (los_type == NLOS_S)
				{
					// Amarillo
					los_marker.color.r = 1.0;
					los_marker.color.g = 1.0;
					los_marker.color.b = 0.0;
				}
				else if (los_type == NLOS_H)
				{
					// Naranja
					los_marker.color.r = 1.0;
					los_marker.color.g = 0.5;
					los_marker.color.b = 0.0;
				}
				else if (los_type == NLOS)
				{
					// Rojo
					los_marker.color.r = 1.0;
					los_marker.color.g = 0.0;
					los_marker.color.b = 0.0;
				}

				marker_array.markers.push_back(los_marker);
			}
		}

		if (markers_pub_) {
			markers_pub_->publish(marker_array);
		}
	}

	std::string GzUwbBeaconPlugin::getIntersection(sim::EntityComponentManager& _ecm, const math::Vector3d& point1, const math::Vector3d& point2, double& distance)
	{
		std::string obstacle_name = "";

		// Calcula dirección y longitud del rayo
		math::Vector3d direction = point2 - point1;
		double ray_length = direction.Length();
		if (ray_length < 1e-6)
		{
			return obstacle_name;
		}

		// Normalizar dirección
		direction = direction / ray_length;

		// Distancia más corta al rayo
		distance = ray_length;

		// Obtener todos los modelos presentes en el mundo
		auto models = _ecm.EntitiesByComponents(sim::components::Model());

		// Iterar sobre todos los modelos
		for (auto& model : models)
		{
			// Obtener el nombre del modelo
			auto name_comp = _ecm.Component<sim::components::Name>(model);
			if (!name_comp)
				continue;
			std::string model_name = name_comp->Data();

			// Saltar ground plane
			if (model_name.find("ground_plane") != std::string::npos)
				continue;

			// Saltar balizas
			if (model_name.find(beacon_prefix_) == 0 || model == model_entity_)
				continue;

			// Obtener la caja del modelo
			math::AxisAlignedBox model_box = getModelBox(_ecm, model);

			// Comprobar si el rayo intersecta con la caja
			auto [intersects, hit_distance, hit_point] = model_box.Intersect(point1, direction, 0.0, ray_length);
			if (intersects && hit_distance < distance)
			{
				distance = hit_distance;
				obstacle_name = model_name;
			}
		}

		return obstacle_name;
	}

	math::AxisAlignedBox GzUwbBeaconPlugin::getModelBox(sim::EntityComponentManager& _ecm, sim::Entity& model)
	{
		math::AxisAlignedBox result;
		bool first_box = true;

		// Obtener la pose del modelo
		auto model_pose_comp = _ecm.Component<sim::components::Pose>(model);
		if (!model_pose_comp)
			return result;

		math::Pose3d model_pose = model_pose_comp->Data();

		// Obtener todos los links del modelo
		auto links = _ecm.ChildrenByComponents(model, sim::components::Link());
		if (links.empty())
			return result;

		// Para cada link, obtener todas las colisiones
		for (const auto& link : links)
		{
			// Obtener la pose del link relativa al modelo
			auto link_pose_comp = _ecm.Component<sim::components::Pose>(link);
			if (!link_pose_comp)
				continue;

			math::Pose3d link_pose_rel = link_pose_comp->Data();

			// Calcular la pose absoluta del link en coordenadas del mundo
			math::Pose3d link_pose_abs = model_pose * link_pose_rel;

			// Obtener todas las colisiones para este link
			auto collisions = _ecm.ChildrenByComponents(link, sim::components::Collision());
			if (collisions.empty())
				continue;

			// Procesar cada colisión
			for (const auto& collision : collisions)
			{
				// Obtener la pose de la colisión relativa al link
				auto collision_pose_comp = _ecm.Component<sim::components::Pose>(collision);
				math::Pose3d collision_pose_rel = collision_pose_comp ?
					collision_pose_comp->Data() : math::Pose3d();

				// Calcular la pose absoluta de la colisión en coordenadas del mundo
				math::Pose3d collision_pose_abs = link_pose_abs * collision_pose_rel;

				// Obtener la geometría de la colisión
				auto geom_comp = _ecm.Component<sim::components::Geometry>(collision);
				if (!geom_comp)
					continue;

				auto geom_data = geom_comp->Data();
				if (geom_data.Type() == sdf::GeometryType::BOX)
				{
					// Obtener el tamaño de la caja
					const sdf::Box* box_shape_sdf = geom_data.BoxShape();
					if (!box_shape_sdf)
						continue;

					math::Vector3d box_size = box_shape_sdf->Size();

					// Crear los vértices de la caja en coordenadas locales
					std::vector<math::Vector3d> vertices;
					vertices.reserve(8);
					for (int i = 0; i < 8; ++i)
					{
						double x = ((i & 1) ? 0.5 : -0.5) * box_size.X();
						double y = ((i & 2) ? 0.5 : -0.5) * box_size.Y();
						double z = ((i & 4) ? 0.5 : -0.5) * box_size.Z();
						vertices.push_back(math::Vector3d(x, y, z));
					}

					// Transformar vértices a coordenadas del mundo y expandir la caja
					math::AxisAlignedBox box;
					for (const auto& vertex : vertices)
					{
						// Transformar el vértice a coordenadas del mundo
						math::Vector3d world_vertex = collision_pose_abs.Pos() +
							collision_pose_abs.Rot().RotateVector(vertex);

						// En el primer vértice de la primera caja, inicializar la caja
						if (first_box && &vertex == &vertices.front())
						{
							box.Min() = world_vertex;
							box.Max() = world_vertex;
							first_box = false;
						}
						else
						{
							// Expandir la caja para incluir este vértice
							box.Min().X() = std::min(box.Min().X(), world_vertex.X());
							box.Min().Y() = std::min(box.Min().Y(), world_vertex.Y());
							box.Min().Z() = std::min(box.Min().Z(), world_vertex.Z());

							box.Max().X() = std::max(box.Max().X(), world_vertex.X());
							box.Max().Y() = std::max(box.Max().Y(), world_vertex.Y());
							box.Max().Z() = std::max(box.Max().Z(), world_vertex.Z());
						}
					}

					// Combinar con la caja resultante
					if (first_box)
					{
						result = box;
						first_box = false;
					}
					else
					{
						result.Min().X() = std::min(result.Min().X(), box.Min().X());
						result.Min().Y() = std::min(result.Min().Y(), box.Min().Y());
						result.Min().Z() = std::min(result.Min().Z(), box.Min().Z());

						result.Max().X() = std::max(result.Max().X(), box.Max().X());
						result.Max().Y() = std::max(result.Max().Y(), box.Max().Y());
						result.Max().Z() = std::max(result.Max().Z(), box.Max().Z());
					}
				}
			}
		}

		return result;
	}

	// Registrar el plugin con Gazebo
	GZ_ADD_PLUGIN(
		GzUwbBeaconPlugin,
		sim::System,
		GzUwbBeaconPlugin::ISystemConfigure,
		GzUwbBeaconPlugin::ISystemPreUpdate,
		GzUwbBeaconPlugin::ISystemReset
	)

} // namespace gz

