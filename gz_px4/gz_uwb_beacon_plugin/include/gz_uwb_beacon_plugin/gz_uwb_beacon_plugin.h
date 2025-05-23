#pragma once

// Standard
#include <random>

// ROS2
#include <rclcpp/rclcpp.hpp>

// Gazebo
// Plugin
#include <gz/plugin/Register.hh>
// Sim
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/Util.hh>
// Common
#include <gz/common/Profiler.hh>
// Math
#include <gz/math/Pose3.hh>
// SDF
#include <sdf/Box.hh>

// TF2
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Mensajes de ROS2
#include <gz_uwb_beacon_msgs/msg/measurement.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace gz
{

    // Clase del plugin
    class GzUwbBeaconPlugin :
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemReset
    {
    public: // Tipos
        enum LineOfSight
        {
            LOS,        // Line of Sight
            NLOS,       // Non Line of Sight
            NLOS_S,     // Non Line of Sight Soft
            NLOS_H      // Non Line of Sight Hard
        };

    public: // Parámetros
        std::string beacon_prefix_;
        double tag_z_offset_;
        double nlos_soft_wall_width_;
        double max_db_distance_;
        double step_db_distance_;
        bool use_parent_as_reference_;
        int tag_id_;

    public: // Datos
        // Generador de números aleatorios
        std::default_random_engine random_generator_;

    public: // Constructor
        GzUwbBeaconPlugin();

    public: // Métodos
        void Configure(const sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
            sim::EntityComponentManager& _ecm, sim::EventManager& _eventMgr) override;
        void Reset(const sim::UpdateInfo& _info, sim::EntityComponentManager& _ecm) override;
        void PreUpdate(const sim::UpdateInfo& _info, sim::EntityComponentManager& _ecm) override;
        std::string getIntersection(sim::EntityComponentManager& _ecm, const gz::math::Vector3d& point1, const gz::math::Vector3d& point2, double& distance);
        gz::math::AxisAlignedBox getModelBox(sim::EntityComponentManager& _ecm, sim::Entity& model);

    private: // Componentes de Gazebo/ROS2
        // Nodo ROS2
        std::shared_ptr<rclcpp::Node> node_;

        // Temporizadores
        rclcpp::Clock ros_clock_;
        std::chrono::steady_clock::duration update_period_;
        std::chrono::steady_clock::time_point last_update_time_;

        // Publicadores
        std::unordered_map<int, rclcpp::Publisher<gz_uwb_beacon_msgs::msg::Measurement>::SharedPtr> measurement_pubs_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

        // Entidades de Gazebo
        sim::Entity world_entity_;      // Entidad del mundo
        sim::Entity model_entity_;      // Entidad del modelo
        sim::Entity tag_link_entity_;   // Entidad del tag

    private: // Constantes
        const double ranging_std_[141][3] =
        {
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.501, -12.192,  22.182},
            {30.698, -12.191,  22.182},
            {27.075, -12.096,  22.17},
            {29.929, -2.799,  21.951},
            {28.478, 33.958,  23.155},
            {25.672, 56.248,  23.111},
            {23.919, 50.137,  24.211},
            {25.466, 35.927,  22.698},
            {23.742, 41.73,  22.97},
            {22.981, 64.983,  21.461},
            {23.656, 98.312,  23.793},
            {27.169, 110.68,  22.425},
            {27.498, 105.53,  21.579},
            {27.299, 85.8,  22.028},
            {25.972, 93.239,  22.068},
            {23.662, 102.09,  24.255},
            {24.762, 105.34,  23.838},
            {24.073, 115.39,  23.856},
            {22.651, 107.15,  23.96},
            {21.012, 133.86, 23.443},
            {25.325, 102.38,  22.679},
            {25.919, 98.105, 22.753},
            {25.403, 124.86,  23.646},
            {25.696, 175.07,  23.149},
            {23.453, 200.68, 23.195},
            {22.291, 205.53,  23.395},
            {22.217, 187.33,  23.42},
            {22.472, 164.73,  23.167},
            {21.454, 142.57,  23.232},
            {21.642, 130.29,  23.18},
            {22.182, 137.13,  22.787},
            {22.852, 119.3,  22.424},
            {23.71, 132.55, 22.891},
            {23.369, 151.35,  23.018},
            {22.471, 149.82,  20.06},
            {21.729, 147.44,  22.383},
            {21.394, 167.7,  23.749},
            {22.086, 172.37, 24.022},
            {20.258, 173.26,  24.311},
            {20.121, 170.93,  24.36},
            {22.828, 177.02,  24.671},
            {22.247, 193.45,  24.648},
            {22.092, 207.94,  24.564},
            {22.288, 196.53,  23.557},
            {22.287, 157.33,  20.073},
            {22.114, 151.18,  24.099},
            {23.635, 149.36,  24.485},
            {22.904, 141.41, 22.277},
            {21.413, 142.79, 19.144},
            {21.388, 162.89,  21.555},
            {20.659, 171.44, 20.923},
            {20.97, 179.38,  22.282},
            {20.82, 189.58,  21.634},
            {19.158, 197.86,  16.7},
            {18.858, 192.62,  16.12},
            {19.446, 169.91,  16.12},
            {19.774, 160.03,  16.12},
            {19.871, 156.27,  16.12},
            {21.165, 155.08,  16.12},
            {20.888, 152.8,  16.12},
            {20.997, 155.59,  16.12},
            {21.589, 160.88,  16.12},
            {22.173, 164.23,  16.12},
            {20.549, 168.01,  16.12},
            {21.734, 168.27,  16.12},
            {23.783, 160.79,  16.12},
            {24.29, 168.02,  16.12},
            {25.308, 183.71,  16.12},
            {24.18, 194.86,  16.12},
            {22.495, 201.82,  16.12},
            {20.357, 206.62,  16.12},
            {18.665, 218.39, 16.12},
            {18.114, 223.67,  16.12},
            {18.784, 222.92,  16.12},
            {20.542, 222.9,  16.12},
            {17.736, 228.38, 16.12},
            {16.072, 226.38,  16.12},
            {15.945, 231.64,  16.12},
            {16.179, 228.45,  16.12},
            {19.054, 225.98, 16.12},
            {19.604, 225.35,  16.12},
            {19.569, 225.8,  16.12},
            {19.414, 226.88, 16.12},
            {19.44, 225.8,  16.12},
            {20.681, 212.01,  16.12},
            {20.904, 205.07,  16.12},
            {20.855, 204.65,  16.12},
            {19.894, 208.66,  16.12},
            {20.67, 216.24,  16.12},
            {21.353, 219.08,  16.12},
            {21.954, 225.47,  16.12},
            {22.372, 230.28,  16.12},
            {21.988, 231.1,  16.12},
            {21.155, 231.92,  16.12},
            {23.487, 236.02,  16.12},
            {28.752, 239.14,  16.12},
            {34.356, 240.34, 16.12},
            {31.388, 241.54,  16.12},
            {25.201, 241.3, 16.12},
            {20.001, 239.64,  16.12},
            {17.451, 238.92,  16.12},
            {17.839, 244.44,  16.12},
            {17.782, 246.21,  16.12},
            {18.332, 246.51,  16.12},
            {21.499, 243.48,  16.12},
            {22.921, 241.79,  16.12},
            {31.445, 241,  16.12},
            {36.097, 239.04,  16.12},
            {33.022, 235.4,  16.12},
            {19.251, 234.77,  16.12},
            {16.86, 235.51,  16.12},
            {16.385, 237.4, 16.127}
        };
        const double rss_mean_[141][3] =
        {
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.88, -80.479, -88.205},
            {-79.506, -80.479, -88.205},
            {-82.328, -80.484, -88.205},
            {-80.104, -81.027, -88.207},
            {-81.234, -83.172, -88.209},
            {-83.418, -84.473, -88.209},
            {-84.783, -84.118, -88.21},
            {-83.582, -83.29, -88.211},
            {-84.924, -83.628, -88.212},
            {-85.516, -84.984, -88.211},
            {-84.992, -86.928, -88.21},
            {-82.254, -87.65, -88.21},
            {-81.997, -87.35, -88.211},
            {-82.154, -86.202, -88.211},
            {-83.19, -86.638, -88.212},
            {-84.989, -87.155, -88.212},
            {-84.141, -87.344, -88.213},
            {-84.674, -87.937, -88.213},
            {-85.777, -87.453, -88.213},
            {-87.053, -89.015, -88.215},
            {-83.692, -87.179, -88.218},
            {-83.228, -86.935, -88.223},
            {-83.63, -88.499, -88.238},
            {-83.403, -91.422, -88.278},
            {-85.147, -92.916, -88.288},
            {-86.052, -93.197, -88.376},
            {-86.111, -92.14, -88.385},
            {-85.914, -90.822, -88.407},
            {-86.704, -89.53, -88.48},
            {-86.559, -88.817, -88.537},
            {-86.14, -89.211, -88.536},
            {-85.622, -88.167, -88.578},
            {-84.957, -88.936, -88.717},
            {-85.223, -90.033, -89.85},
            {-85.919, -89.94, -92.143},
            {-86.493, -89.8, -91.798},
            {-86.751, -90.982, -89.445},
            {-86.212, -91.251, -89.327},
            {-87.634, -91.302, -89.909},
            {-87.741, -91.167, -89.869},
            {-85.638, -91.523, -90.272},
            {-86.092, -92.484, -90.161},
            {-86.212, -93.332, -90.502},
            {-86.061, -92.669, -93.029},
            {-86.065, -90.385, -93.384},
            {-86.211, -90.028, -91.776},
            {-85.035, -89.922, -90.725},
            {-85.606, -89.462, -90.974},
            {-86.763, -89.549, -92.572},
            {-86.782, -90.725, -92.237},
            {-87.356, -91.241, -92.515},
            {-87.129, -91.699, -92.342},
            {-87.265, -92.289, -92.127},
            {-88.563, -92.766, -93.68},
            {-88.806, -92.453, -91.294},
            {-88.337, -91.124, -89.336},
            {-88.071, -90.549, -89.039},
            {-87.99, -90.324, -89.615},
            {-87.001, -90.262, -92.496},
            {-87.184, -90.132, -94.345},
            {-87.092, -90.289, -92.867},
            {-86.638, -90.592, -92.249},
            {-86.202, -90.785, -92.691},
            {-87.46, -91.007, -93.252},
            {-86.553, -91.026, -94.637},
            {-84.991, -90.587, -93.508},
            {-84.645, -91.012, -93.939},
            {-83.922, -91.933, -91.488},
            {-84.876, -92.586, -91.667},
            {-86.233, -92.996, -94.21},
            {-87.94, -93.285, -95.191},
            {-89.02, -93.994, -97.303},
            {-89.428, -94.328, -96.513},
            {-88.962, -94.278, -98.077},
            {-87.743, -94.27, -97.932},
            {-89.904, -94.627, -95.275},
            {-91.299, -94.532, -96.202},
            {-91.45, -94.877, -99.615},
            {-91.288, -94.683, -100.46},
            {-89.362, -94.536, -98.965},
            {-88.952, -94.506, -97.864},
            {-89.241, -94.537, -98.488},
            {-89.77, -94.59, -98.489},
            {-90.054, -94.547, -99.341},
            {-90.534, -93.698, -100.08},
            {-89.67, -93.28, -101.08},
            {-89.052, -93.243, -102.03},
            {-89.73, -93.471, -102.45},
            {-91.068, -93.958, -103.27},
            {-90.93, -94.179, -103.62},
            {-90.164, -94.583, -103.59},
            {-88.864, -94.875, -103.87},
            {-88.026, -94.923, -104.27},
            {-90.477, -94.978, -104.42},
            {-92.332, -95.251, -104.45},
            {-93.692, -95.463, -104.55},
            {-94.745, -95.582, -104.56},
            {-94.266, -95.66, -104.63},
            {-93.197, -95.654, -104.6},
            {-92.248, -95.547, -104.62},
            {-91.703, -95.476, -104.61},
            {-91.789, -95.834, -104.62},
            {-91.774, -95.963, -104.72},
            {-91.943, -95.998, -104.74},
            {-92.598, -95.763, -104.74},
            {-92.857, -95.634, -104.74},
            {-94.314, -95.57, -104.74},
            {-95.107, -95.408, -104.74},
            {-94.594, -95.116, -104.74},
            {-92.277, -95.087, -104.74},
            {-91.883, -95.129, -104.74},
            {-91.802, -95.257, -104.74}
        };
        const double rss_std_[141][3] =
        {
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28273, 0.52405, 0.42595},
            {0.28802, 0.52405, 0.42595},
            {0.31193, 0.52392, 0.42593},
            {0.29309, 0.51064, 0.42581},
            {0.30265, 0.45871, 0.42568},
            {0.32115, 0.42762, 0.42565},
            {0.33271, 0.43817, 0.42562},
            {0.32256, 0.45904, 0.4256},
            {0.33391, 0.4501, 0.42562},
            {0.33892, 0.41617, 0.4256},
            {0.33449, 0.36839, 0.42557},
            {0.3113, 0.35052, 0.42555},
            {0.30912, 0.35914, 0.42558},
            {0.31046, 0.39002, 0.42557},
            {0.31924, 0.38186, 0.4256},
            {0.33449, 0.36936, 0.42561},
            {0.32735, 0.36462, 0.42561},
            {0.33184, 0.35698, 0.42562},
            {0.34116, 0.36488, 0.42563},
            {0.35196, 0.33029, 0.42565},
            {0.32349, 0.37679, 0.42564},
            {0.31955, 0.38847, 0.42567},
            {0.32295, 0.35443, 0.42579},
            {0.32104, 0.27502, 0.42622},
            {0.33579, 0.23774, 0.42635},
            {0.34345, 0.22965, 0.42749},
            {0.34396, 0.26009, 0.4276},
            {0.3423, 0.29305, 0.42789},
            {0.34898, 0.32595, 0.42884},
            {0.34775, 0.34724, 0.42957},
            {0.34422, 0.33149, 0.42957},
            {0.33985, 0.35423, 0.43011},
            {0.33424, 0.3302, 0.43196},
            {0.3365, 0.30287, 0.44692},
            {0.34237, 0.30164, 0.47716},
            {0.34721, 0.30319, 0.47259},
            {0.34937, 0.27464, 0.44155},
            {0.34481, 0.26435, 0.44},
            {0.35685, 0.26219, 0.4477},
            {0.35775, 0.26625, 0.44715},
            {0.33998, 0.25858, 0.45245},
            {0.34383, 0.23685, 0.451},
            {0.34485, 0.21927, 0.4555},
            {0.34357, 0.23866, 0.48884},
            {0.34362, 0.29779, 0.49354},
            {0.34493, 0.30859, 0.47231},
            {0.33503, 0.31208, 0.45842},
            {0.33986, 0.32803, 0.46171},
            {0.34964, 0.33222, 0.48286},
            {0.34979, 0.30749, 0.47844},
            {0.35469, 0.31324, 0.48211},
            {0.35286, 0.29601, 0.47985},
            {0.35413, 0.27666, 0.47704},
            {0.36514, 0.2577, 0.49755},
            {0.36726, 0.25683, 0.46604},
            {0.36322, 0.28514, 0.4402},
            {0.3609, 0.30088, 0.43625},
            {0.36018, 0.30122, 0.44384},
            {0.35193, 0.30984, 0.48189},
            {0.35328, 0.31716, 0.50633},
            {0.35245, 0.30678, 0.48683},
            {0.34865, 0.29261, 0.4787},
            {0.34507, 0.28513, 0.48469},
            {0.35568, 0.28172, 0.49213},
            {0.34809, 0.28503, 0.51043},
            {0.33508, 0.29321, 0.49557},
            {0.33243, 0.28578, 0.50118},
            {0.32673, 0.26951, 0.46866},
            {0.33526, 0.25688, 0.47096},
            {0.34701, 0.25141, 0.50455},
            {0.36172, 0.25416, 0.51752},
            {0.36944, 0.26108, 0.54557},
            {0.37277, 0.28157, 0.53503},
            {0.36915, 0.27648, 0.55569},
            {0.35973, 0.26925, 0.55374},
            {0.37788, 0.30123, 0.51851},
            {0.39029, 0.32821, 0.53077},
            {0.39187, 0.36244, 0.57602},
            {0.39062, 0.35787, 0.58723},
            {0.37618, 0.35901, 0.56732},
            {0.37281, 0.36604, 0.55273},
            {0.37682, 0.37148, 0.56097},
            {0.38374, 0.35877, 0.56095},
            {0.38796, 0.38246, 0.57229},
            {0.40064, 0.35333, 0.58209},
            {0.38921, 0.34942, 0.59541},
            {0.38007, 0.33631, 0.60813},
            {0.38539, 0.32452, 0.61413},
            {0.40828, 0.36174, 0.62553},
            {0.40946, 0.41779, 0.63079},
            {0.40121, 0.44286, 0.63038},
            {0.3844, 0.44835, 0.63433},
            {0.37052, 0.44675, 0.64188},
            {0.40202, 0.45388, 0.64658},
            {0.43959, 0.48541, 0.6486},
            {0.48364, 0.51261, 0.65211},
            {0.52483, 0.565, 0.65087},
            {0.50416, 0.57133, 0.65671},
            {0.46002, 0.5807, 0.6518},
            {0.42221, 0.57222, 0.65058},
            {0.40252, 0.54167, 0.64913},
            {0.40556, 0.57271, 0.64843},
            {0.40507, 0.59777, 0.65558},
            {0.41007, 0.61706, 0.65859},
            {0.43421, 0.55792, 0.6586},
            {0.44454, 0.52705, 0.6586},
            {0.5051, 0.50872, 0.6586},
            {0.53813, 0.45956, 0.6586},
            {0.51647, 0.37797, 0.6586},
            {0.41915, 0.38748, 0.6586},
            {0.40237, 0.38546, 0.6586},
            {0.39899, 0.40199, 0.6586}
        };
        const double min_power_[3] = { -94.5, -94.5, -94.5 };
        const double ranging_offset_[141][2] =
        {
            {-87.505, 228.73},
            {-86.505, 229.73},
            {-85.505, 230.73},
            {-84.505, 231.73},
            {-83.505, 232.73},
            {-82.505, 233.73},
            {-81.505, 234.73},
            {-80.505, 235.73},
            {-79.505, 236.73},
            {-78.505, 237.73},
            {-77.505, 238.73},
            {-76.505, 239.73},
            {-75.505, 240.73},
            {-74.505, 241.73},
            {-73.505, 242.73},
            {-72.505, 243.73},
            {-71.505, 244.73},
            {-70.505, 245.73},
            {-69.505, 246.73},
            {-68.505, 247.73},
            {-67.505, 248.73},
            {-66.505, 249.73},
            {-65.505, 250.73},
            {-64.505, 251.73},
            {-63.505, 252.73},
            {-62.505, 253.73},
            {-61.505, 254.73},
            {-60.505, 255.73},
            {-59.505, 256.73},
            {-58.505, 257.73},
            {-57.505, 258.73},
            {-56.505, 259.73},
            {-171.73, 159.43},
            {19.236, 37.667},
            {-59.397, -69.571},
            {-88.819, 55.978},
            {-87.637, 126.88},
            {-63.632, 218.34},
            {-166.5, 243.98},
            {2.9297, -133.46},
            {24.966, -7.3586},
            {-75.633, 179.6},
            {-275.63, 177.37},
            {-6.0524, 35.31},
            {-42.925, 2.8818},
            {-91.347, 64.352},
            {-45.604, -11.544},
            {-80.352, -163.65},
            {-70.501, -122.94},
            {-44.816, 47.885},
            {-62.616, 23.613},
            {-120.1, 96.477},
            {-24.175, -54.87},
            {-31.297, 52.603},
            {-68.891, 139.32},
            {-134.92, -112.3},
            {-82.555, 65.72},
            {-99.757, -115.76},
            {-91.724, -272.29},
            {-98.396, 104.96},
            {-135.68, 22.346},
            {-69.928, -142.3},
            {-54.738, -169.96},
            {-65.137, 108.48},
            {-159.24, -85.515},
            {-59.853, 127.5},
            {-37.471, -123.05},
            {-25.802, 25.88},
            {-26.242, -68.14},
            {-169.91, -187.94},
            {-146.91, -169},
            {-194.92, 200.43},
            {-117.44, 160.1},
            {48.046, 62.7},
            {80.894, -50.68},
            {-190.6, -75.238},
            {-360.33, -64.185},
            {-196.96, 0.028312},
            {-67.649, -12.525},
            {40.599, -64.741},
            {3.6309, -128.47},
            {-68.189, -63.786},
            {56.098, -296.78},
            {-44.121, -68.179},
            {-180.8, -82.837},
            {-253.97, 16.43},
            {-76.724, -76.724},
            {63.682, 63.682},
            {208.32, 208.32},
            {147.39, 147.39},
            {-148.67, -148.67},
            {-24.853, -24.853},
            {-15.417, -15.417},
            {-11.058, -11.058},
            {-58.758, -58.758},
            {-275.52, -275.52},
            {114.79, 114.79},
            {177.85, 177.85},
            {137.17, 137.17},
            {76.573, 76.573},
            {39.708, 39.708},
            {-39.521, -39.521},
            {-58.715, -58.715},
            {17.466, 17.466},
            {91.571, 91.571},
            {-14.972, -14.972},
            {-74.248, -74.248},
            {-104.1, -104.1},
            {-188.11, -188.11},
            {-258.89, -258.89},
            {-306.62, -306.62},
            {-257.13, -257.13},
            {-85.931, -85.931},
            {3.3227, 3.3227},
            {59.19, 59.19},
            {3.4495, 3.4495},
            {-29.723, -29.723},
            {9.6787, 9.6787},
            {23.18, 23.18},
            {90.738, 90.738},
            {89.798, 89.798},
            {-14.135, -14.135},
            {-87.788, -87.788},
            {-136.37, -136.37},
            {-174.65, -174.65},
            {-214.81, -214.81},
            {-208.3, -208.3},
            {-218.2, -218.2},
            {-110.18, -110.18},
            {10.234, 10.234},
            {16.536, 16.536},
            {-156.6, -156.6},
            {-58.566, -58.566},
            {69.13, 69.13},
            {136.01, 136.01},
            {-72.069, -72.069},
            {-114.48, -114.48},
            {-89.969, -89.969},
            {-75.265, -75.265},
            {-107.16, -107.16},
            {-224.83, -224.83}
        };
    };

} // namespace gz