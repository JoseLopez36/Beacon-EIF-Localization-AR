#include "gz_uwb_beacon_plugin/gz_uwb_beacon_plugin.h"

namespace gz
{

	GzUwbBeaconPlugin::GzUwbBeaconPlugin()
	{
		// Inicializar el periodo de actualización
		update_period_ = std::chrono::nanoseconds(0);
		// Initialize ROS2 clock
		ros_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);
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

		// Obtener pose del modelo
		auto model_pose_gz_comp = _ecm.Component<sim::components::Pose>(model_entity_);
		if (model_pose_gz_comp)
			model_pose_ = model_pose_gz_comp->Data();
		else
			RCLCPP_FATAL(node_->get_logger(), "UWB-Beacon-Plugin: Error, model pose not found");

		// Obtener parámetros del plugin
		double update_rate = _sdf->Get<double>("update_rate");
		update_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / update_rate));
		beacon_prefix_ = "uwb_beacon_";
		tag_id_ = 0;
		nlos_soft_wall_width_ = 0.25;
		tag_z_offset_ = 0;
		max_db_distance_ = 14;
		step_db_distance_ = 0.1;
		use_parent_as_reference_ = false;

		if (_sdf->HasElement("beacon_prefix"))
		{
			beacon_prefix_ = _sdf->Get<std::string>("beacon_prefix");
		}

		if (_sdf->HasElement("tag_link"))
		{
			std::string tag_link = _sdf->Get<std::string>("tag_link");

			// Buscar el enlace por nombre
			auto links = _ecm.EntitiesByComponents(
				sim::components::Link(),
				sim::components::ParentEntity(model_entity_));

			for (const auto& link : links)
			{
				auto name_comp = _ecm.Component<sim::components::Name>(link);
				if (name_comp && name_comp->Data() == tag_link)
				{
					tag_link_entity_ = link;
					break;
				}
			}

			RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: Looking for tag link (%s)", tag_link.c_str());
			if (tag_link_entity_ == sim::kNullEntity)
			{
				RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: Tag link Is NULL. We use The Parent As Reference");
				use_parent_as_reference_ = true;
			}
		}

		if (_sdf->HasElement("tag_id"))
		{
			tag_id_ = _sdf->Get<int>("tag_id");
		}

		if (_sdf->HasElement("nlosSoftWallWidth"))
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

		// Crear publicador de datos de ranging
		std::string ranging_topic = "/sensors/uwb_beacon/ranging";
		uwb_ranging_pub_ = node_->create_publisher<std_msgs::msg::Float64>(ranging_topic, 100);
		RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: Ranging Publishing in %s", ranging_topic.c_str());

		// Crear publicador de datos de anchors
		std::string anchors_topic = "/sensors/uwb_beacon/anchors";
		uwb_anchors_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(anchors_topic, 100);
		RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: Anchors Position Publishing in %s", anchors_topic.c_str());
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
			tag_pose = tag_pose_comp->Data();
		}
		else
		{
			tag_pose = model_pose_;
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
						}
						else
						{
							// No podemos llegar a la baliza, no hay ranging
							los_type = NLOS;
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

					int index_ranging_offset = (int)round(distance_after_rebounds / step_db_distance_);

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
						std_msgs::msg::Float64 ranging_msg;
						// ranging_msg.beacon_id = bid;
						// ranging_msg.tag_id = tag_id_;
						ranging_msg.data = ranging_value;
						// ranging_msg.rss = power_value;
						// ranging_msg.error_estimation = 0.00393973;
						uwb_ranging_pub_->publish(ranging_msg);
					}
				}

				// Crear el marcador de la baliza
				visualization_msgs::msg::Marker marker;
				marker.header.frame_id = "map";
				marker.header.stamp = ros_clock_.now();
				marker.id = bid;
				marker.type = visualization_msgs::msg::Marker::CYLINDER;
				marker.action = visualization_msgs::msg::Marker::ADD;
				marker.pose.position.x = beacon_pose.Pos().X();
				marker.pose.position.y = beacon_pose.Pos().Y();
				marker.pose.position.z = beacon_pose.Pos().Z();
				marker.pose.orientation.x = beacon_pose.Rot().X();
				marker.pose.orientation.y = beacon_pose.Rot().Y();
				marker.pose.orientation.z = beacon_pose.Rot().Z();
				marker.pose.orientation.w = beacon_pose.Rot().W();
				marker.scale.x = 0.2;
				marker.scale.y = 0.2;
				marker.scale.z = 0.5;
				marker.color.a = 1.0;

				// Colores de los marcadores según el tipo de Line-Of-Sight
				if (los_type == LOS)
				{
					marker.color.r = 0.0;
					marker.color.g = 0.6;
					marker.color.b = 0.0;
				}
				else if (los_type == NLOS_S)
				{
					marker.color.r = 0.6;
					marker.color.g = 0.6;
					marker.color.b = 0.0;
				}
				else if (los_type == NLOS_H)
				{
					marker.color.r = 0.0;
					marker.color.g = 0.0;
					marker.color.b = 0.6;
				}
				else if (los_type == NLOS)
				{
					marker.color.r = 0.6;
					marker.color.g = 0.0;
					marker.color.b = 0.0;
				}

				marker_array.markers.push_back(marker);
			}
		}

		uwb_anchors_pub_->publish(marker_array);
	}

	std::string GzUwbBeaconPlugin::getIntersection(sim::EntityComponentManager& _ecm, const math::Vector3d& point1, const math::Vector3d& point2, double& distance)
	{
		std::string obstacle_name = "";
		distance = 0.0;

		// Calculate direction and length
		math::Vector3d direction = point2 - point1;
		double ray_length = direction.Length();

		// Check if points are too close
		if (ray_length < 1e-6)
		{
			return obstacle_name;
		}

		// Normalize direction
		direction = direction / ray_length;

		// Get all models with collision components
		auto models = _ecm.EntitiesByComponents(sim::components::Model(),
			sim::components::Collision());

		double closest_hit_distance = ray_length;

		for (const auto& model : models)
		{
			auto name_comp = _ecm.Component<sim::components::Name>(model);
			if (!name_comp)
				continue;

			// Skip beacons or other models we don't want to check
			if (name_comp->Data().find(beacon_prefix_) == 0)
				continue;

			// Get model pose
			auto model_pose_comp = _ecm.Component<sim::components::Pose>(model);
			if (!model_pose_comp)
				continue;

			math::Pose3d model_pose = model_pose_comp->Data();

			// Simplified collision check - treat all models as spheres with radius 1.0
			// This is a rough approximation - in a real plugin we would use proper collision geometry
			double radius = 1.0;

			// Vector from ray origin to sphere center
			math::Vector3d origin_to_center = model_pose.Pos() - point1;

			// Project this vector onto the ray direction
			double projection = origin_to_center.Dot(direction);

			// If the projection is negative, the sphere is behind the ray origin
			if (projection < 0)
				continue;

			// If the projection is greater than the ray length, the sphere is beyond the ray end
			if (projection > ray_length)
				continue;

			// Find the closest point on the ray to the sphere center
			math::Vector3d closest_point = point1 + direction * projection;

			// Calculate the distance from the closest point to the sphere center
			double closest_distance = (closest_point - model_pose.Pos()).Length();

			// If this distance is less than the sphere radius, we have an intersection
			if (closest_distance <= radius && projection < closest_hit_distance)
			{
				closest_hit_distance = projection;
				obstacle_name = name_comp->Data();
			}
		}

		if (obstacle_name != "")
		{
			distance = closest_hit_distance;
		}

		return obstacle_name;
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

