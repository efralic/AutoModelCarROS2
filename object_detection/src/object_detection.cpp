// ─────────────────────────────────────────────────────────────────────────────
//  object_detection.cpp  –  ROS 2 (Humble / Iron)
//  Sensor: Orbbec Astra Pro  (reemplaza LIDAR LD14P / RPLIDAR C1)
//
//  CAMBIO PRINCIPAL:
//    Antes  → suscribe /scan (sensor_msgs::LaserScan)
//             lee ángulo + distancia en coordenadas polares 360°
//    Ahora  → suscribe /camera/depth/image_raw (sensor_msgs::Image, 16UC1)
//             lee imagen de profundidad en mm → convierte a distancia + ángulo
//
//  INVARIANTE (lo que NO cambia):
//    • Algoritmo DBSCAN exactamente igual
//    • transform_point exactamente igual
//    • Formato de salida /objects_points exactamente igual
//    • Los Masters NO necesitan modificarse
//
//  Sistema de coordenadas de salida (obs.x, obs.y):
//    obs.x = distancia al centroide del obstáculo  [cm]
//    obs.y = ángulo al centroide del obstáculo     [grados, 0-360°]
//            90° = frente exacto del carrito
//            <90° = obstáculo a la izquierda
//            >90° = obstáculo a la derecha
//
//  Mapeo píxel → ángulo (Astra Pro, FOV_H = 60°):
//    ángulo = 90° + (pixel_x - cx) × (FOV_H / image_width)
//    donde cx = 320  (centro óptico horizontal)
//    Rango resultante: 60° (borde izquierdo) ... 120° (borde derecho)
//    → Exactamente el rango que condicionan los Masters para "obstáculo al frente"
//
//  Migración ROS 1 → ROS 2:
//    ros::NodeHandle          → herencia de rclcpp::Node
//    nh.subscribe/advertise   → create_subscription / create_publisher
//    sensor_msgs::LaserScan   → sensor_msgs::msg::Image (profundidad 16UC1)
//    object_detection::points_objects  → object_detection::msg::PointsObjects
//    ROS_INFO                 → RCLCPP_INFO(get_logger(), ...)
//    ros::init + ros::spin    → rclcpp::init + rclcpp::spin
// ─────────────────────────────────────────────────────────────────────────────

#include <cmath>
#include <map>
#include <vector>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "object_detection/msg/points_objects.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

// ── Constantes DBSCAN ────────────────────────────────────────────────────────
#define UNCLASSIFIED -1
#define CORE_POINT    1
#define BORDER_POINT  2
#define NOISE        -2
#define SUCCESS       0
#define FAILURE      -3

// ── Parámetros de la Orbbec Astra Pro ────────────────────────────────────────
static constexpr float ASTRA_FOV_H_DEG  = 60.0f;  // campo de visión horizontal
static constexpr int   ASTRA_WIDTH      = 640;     // resolución horizontal
static constexpr int   ASTRA_HEIGHT     = 480;     // resolución vertical
static constexpr float ASTRA_CX        = 320.0f;  // centro óptico horizontal
static constexpr float ASTRA_CY        = 240.0f;  // centro óptico vertical

// Rango de detección (en mm, porque la Astra Pro publica en mm)
static constexpr float RANGE_MIN_MM    = 600.0f;  // 0.6 m = rango mínimo Astra
static constexpr float RANGE_MAX_MM    = 4000.0f; // 4.0 m = rango máximo útil

// Submuestreo: procesar 1 de cada N píxeles para reducir carga computacional
static constexpr int   DEPTH_STEP      = 8;

// ── DBSCAN ───────────────────────────────────────────────────────────────────
class PointC {
public:
    float x, y;
    int   clusterID;
    PointC(float x_, float y_) : x(x_), y(y_), clusterID(UNCLASSIFIED) {}
};

class DBSCAN {
public:
    std::vector<PointC> m_points;

    DBSCAN(unsigned int minPts, float eps,
           const std::vector<cv::Point> & points_in)
    : m_minPoints(minPts), m_epsilon(eps)
    {
        for (auto & p : points_in)
            m_points.emplace_back((float)p.x, (float)p.y);
        m_pointSize = m_points.size();
    }

    int run() {
        int clusterID = 0;
        for (auto & pt : m_points)
            if (pt.clusterID == UNCLASSIFIED)
                if (expandCluster(pt, clusterID) != FAILURE)
                    clusterID++;
        return 0;
    }

    void getCluster(std::map<int, std::vector<cv::Point>> & out) {
        for (auto & p : m_points)
            out[p.clusterID].push_back(cv::Point((int)p.x, (int)p.y));
    }

private:
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float        m_epsilon;

    inline double calculateDistance(const PointC & a, const PointC & b) {
        return std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
    }

    std::vector<int> calculateCluster(const PointC & point) {
        std::vector<int> idx;
        for (int i = 0; i < (int)m_points.size(); i++)
            if (calculateDistance(point, m_points[i]) <= m_epsilon)
                idx.push_back(i);
        return idx;
    }

    int expandCluster(PointC & point, int clusterID) {
        std::vector<int> seeds = calculateCluster(point);
        if (seeds.size() < m_minPoints) { point.clusterID = NOISE; return FAILURE; }

        int indexCore = 0, index = 0;
        for (auto idx : seeds) {
            m_points[idx].clusterID = clusterID;
            if (m_points[idx].x == point.x && m_points[idx].y == point.y)
                indexCore = index;
            index++;
        }
        seeds.erase(seeds.begin() + indexCore);

        for (size_t i = 0, n = seeds.size(); i < n; i++) {
            auto neighbors = calculateCluster(m_points[seeds[i]]);
            if (neighbors.size() >= m_minPoints) {
                for (auto ni : neighbors) {
                    if (m_points[ni].clusterID == UNCLASSIFIED ||
                        m_points[ni].clusterID == NOISE) {
                        if (m_points[ni].clusterID == UNCLASSIFIED) {
                            seeds.push_back(ni);
                            n = seeds.size();
                        }
                        m_points[ni].clusterID = clusterID;
                    }
                }
            }
        }
        return SUCCESS;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  ObjectDetection – nodo ROS 2
// ─────────────────────────────────────────────────────────────────────────────
class ObjectDetection : public rclcpp::Node
{
public:
    ObjectDetection() : rclcpp::Node("ObjectDetection")
    {
        // ── Parámetros configurables ──────────────────────────────────────
        this->declare_parameter("minimum_points",  1);
        this->declare_parameter("epsilon",         95);
        this->declare_parameter("depth_topic",     "/camera/depth/image_raw");
        this->declare_parameter("depth_step",      DEPTH_STEP);

        MINIMUM_POINTS_ = this->get_parameter("minimum_points").as_int();
        EPSILON_        = this->get_parameter("epsilon").as_int();
        depth_step_     = this->get_parameter("depth_step").as_int();
        std::string depth_topic = this->get_parameter("depth_topic").as_string();

        // ── Publicador ────────────────────────────────────────────────────
        // ROS 1: nh.advertise<object_detection::points_objects>("objects_points", 1)
        // ROS 2: create_publisher<object_detection::msg::PointsObjects>(...)
        pub_ = this->create_publisher<object_detection::msg::PointsObjects>(
                   "objects_points", 1);

        // ── Subscriptor a imagen de profundidad de la Astra Pro ───────────
        // ROS 1: nh.subscribe("/scan", 1, &ObjectDetection::laser_msg_Callback, this)
        // ROS 2: create_subscription<sensor_msgs::msg::Image>(depth_topic, ...)
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, 1,
            std::bind(&ObjectDetection::depth_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "ObjectDetection listo. Escuchando profundidad: %s [Orbbec Astra Pro]",
            depth_topic.c_str());
        RCLCPP_INFO(get_logger(),
            "Rango: %.0f – %.0f mm | DBSCAN eps=%d minPts=%d | paso=%d px",
            RANGE_MIN_MM, RANGE_MAX_MM, EPSILON_, MINIMUM_POINTS_, depth_step_);
    }

private:
    // ── Miembros ──────────────────────────────────────────────────────────
    rclcpp::Publisher<object_detection::msg::PointsObjects>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           depth_sub_;

    int MINIMUM_POINTS_;
    int EPSILON_;
    int depth_step_;

    std::vector<cv::Point>                  points_;
    std::map<int, std::vector<cv::Point>>   clusters_points_;
    std::vector<cv::Point>                  points_centroids_;

    // ─────────────────────────────────────────────────────────────────────
    //  depth_callback  (reemplaza laser_msg_Callback)
    //
    //  La Astra Pro publica depth como sensor_msgs::msg::Image con:
    //    encoding = "16UC1"   (uint16, valores en milímetros)
    //    width    = 640
    //    height   = 480
    //
    //  Conversión píxel → ángulo:
    //    ángulo_deg = 90° + (u - CX) × (FOV_H / WIDTH)
    //               = 90° + (u - 320) × 0.09375°/px
    //
    //  Luego se reutiliza EXACTAMENTE el mismo pipeline que con el LIDAR:
    //    (ángulo_deg, distancia_cm) → cartesiano 800×800 → DBSCAN → centroide
    //    → transform_point → (obs.x=distancia, obs.y=ángulo)
    // ─────────────────────────────────────────────────────────────────────
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Verificar encoding
        if (msg->encoding != "16UC1") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Encoding inesperado: %s (esperado 16UC1)", msg->encoding.c_str());
            return;
        }

        int width  = (int)msg->width;
        int height = (int)msg->height;
        float cx   = width  / 2.0f;
        float fov_per_pixel = ASTRA_FOV_H_DEG / (float)width;  // °/px ≈ 0.09375

        // Puntos válidos en coordenadas (ángulo_deg, distancia_cm)
        std::vector<std::pair<float,float>> scan_points;  // (angle_deg, range_cm)

        const uint16_t* row_ptr = nullptr;
        for (int v = 0; v < height; v += depth_step_) {
            row_ptr = reinterpret_cast<const uint16_t*>(
                          msg->data.data() + v * msg->step);
            for (int u = 0; u < width; u += depth_step_) {
                uint16_t depth_mm = row_ptr[u];

                // Filtrar valores fuera de rango o inválidos (0 = sin dato)
                if (depth_mm == 0) continue;
                if ((float)depth_mm < RANGE_MIN_MM) continue;
                if ((float)depth_mm > RANGE_MAX_MM) continue;

                // ── Píxel → ángulo (sistema polar del carrito) ────────────
                //   u=0   → ángulo = 90° - FOV/2 = 60°  (izquierda)
                //   u=320 → ángulo = 90°               (frente exacto)
                //   u=640 → ángulo = 90° + FOV/2 = 120° (derecha)
                float angle_deg = 90.0f + (u - cx) * fov_per_pixel;

                // Profundidad mm → distancia en cm (mismo sistema que el LIDAR)
                float range_cm  = (float)depth_mm / 10.0f;

                scan_points.emplace_back(angle_deg, range_cm);
            }
        }

        if (scan_points.empty()) return;

        // ── Pipeline DBSCAN (idéntico al original con LIDAR) ─────────────
        get_object_points(scan_points);

        // ── Publicar si hay obstáculos ────────────────────────────────────
        if (!points_centroids_.empty()) {
            object_detection::msg::PointsObjects points_msg;
            points_msg.another_field = (uint8_t)points_centroids_.size();

            for (auto & centroid : points_centroids_) {
                geometry_msgs::msg::Point pt;
                transform_point(centroid, pt);
                pt.z = 0.0;
                points_msg.points.push_back(pt);
                RCLCPP_INFO(get_logger(),
                    "Obstáculo: dist=%.1f cm, angulo=%.1f deg",
                    pt.x, pt.y);
            }
            pub_->publish(points_msg);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    //  get_object_points  (mismo algoritmo que antes, solo cambia el input)
    //
    //  Convierte (ángulo_deg, rango_cm) → punto cartesiano en imagen 800×800
    //  usando la misma fórmula que usaba el código original con el LIDAR.
    //  Esto hace que DBSCAN y transform_point funcionen exactamente igual.
    // ─────────────────────────────────────────────────────────────────────
    void get_object_points(const std::vector<std::pair<float,float>> & scan_points)
    {
        points_.clear();
        clusters_points_.clear();

        for (auto & [angle_deg, range_cm] : scan_points) {
            // Misma conversión polar → cartesiano del código original:
            // x = 400 + r·cos((180° - ángulo)·π/180)
            // y = 400 + r·sin((180° - ángulo)·π/180)
            float r = range_cm;
            float rad = (180.0f - angle_deg) * (float)M_PI / 180.0f;
            float x = 400.0f + r * std::cos(rad);
            float y = 400.0f + r * std::sin(rad);

            x = std::max(0.0f, std::min(800.0f, x));
            y = std::max(0.0f, std::min(800.0f, y));
            points_.emplace_back((int)x, (int)y);
        }

        if (points_.empty()) return;

        DBSCAN dbScan(MINIMUM_POINTS_, (float)EPSILON_, points_);
        dbScan.run();
        dbScan.getCluster(clusters_points_);
        get_centroids_objects(clusters_points_);
    }

    // ─────────────────────────────────────────────────────────────────────
    //  transform_point  (idéntico al original – sin cambios)
    //
    //  Convierte centroide en imagen 800×800 → (distancia_cm, ángulo_deg)
    //  que es lo que leen los Masters en obs.x y obs.y
    // ─────────────────────────────────────────────────────────────────────
    void transform_point(const cv::Point & point,
                         geometry_msgs::msg::Point & point_msg)
    {
        float adj = (float)(point.y - 400);
        float opp = (float)(point.x - 400);
        if (adj == 0.0f) adj = 0.000001f;

        if      (point.x >= 0   && point.x <= 400 && point.y >= 0   && point.y <= 400)
            point_msg.y = std::atan(opp / adj) * 57.295779f;
        else if (point.x >= 0   && point.x <= 400 && point.y >= 400 && point.y <= 800)
            point_msg.y = 180.0f + std::atan(opp / adj) * 57.295779f;
        else if (point.x >= 400 && point.x <= 800 && point.y >= 400 && point.y <= 800)
            point_msg.y = 180.0f + std::atan(opp / adj) * 57.295779f;
        else if (point.x >= 400 && point.x <= 800 && point.y >= 0   && point.y <= 400)
            point_msg.y = 360.0f + std::atan(opp / adj) * 57.295779f;

        if (point_msg.y < 0) point_msg.y = -point_msg.y;
        point_msg.x = cv::norm(point - cv::Point(400, 400));
    }

    // ─────────────────────────────────────────────────────────────────────
    //  get_centroids_objects  (idéntico al original – sin cambios)
    // ─────────────────────────────────────────────────────────────────────
    void get_centroids_objects(
        const std::map<int, std::vector<cv::Point>> & cp)
    {
        points_centroids_.clear();
        for (auto & [id, cluster] : cp) {
            if (id == NOISE || cluster.empty()) continue;
            int sx = 0, sy = 0;
            for (auto & p : cluster) { sx += p.x; sy += p.y; }
            int n = (int)cluster.size();
            points_centroids_.emplace_back(sx / n, sy / n);
        }
    }
};


// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    // ROS 1: ros::init + new ObjectDetection() + ros::spin()
    // ROS 2: rclcpp::init + make_shared<Node> + rclcpp::spin
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetection>());
    rclcpp::shutdown();
    return 0;
}
