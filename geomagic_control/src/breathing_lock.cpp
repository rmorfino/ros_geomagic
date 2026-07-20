#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geomagic_control/DeviceFeedback.h>
#include <geometry_msgs/PointStamped.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <atomic>

static geometry_msgs::PoseStamped g_pose;
static bool g_have_pose = false;

// Flag condiviso tra main thread e keyboard thread
static std::atomic<bool> g_sine_started(false);

// Thread che aspetta INVIO da tastiera
void keyboardThread()
{
    ROS_INFO("breathing_lock: premi INVIO per avviare la sinusoide...");
    std::string line;
    std::getline(std::cin, line);
    g_sine_started.store(true);
    ROS_INFO("breathing_lock: INVIO ricevuto, sinusoide avviata!");
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    g_pose = *msg;
    g_have_pose = true;
}

static double clamp(double v, double lo, double hi)
{
    return std::max(lo, std::min(v, hi));
}

// filtro 1° ordine: y <- y + alpha*(x - y)
static double lpf(double y, double x, double alpha)
{
    return y + alpha * (x - y);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "geomagic_breathing_cylindrical_constraint");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // =========================================================
    // PARAMETRI GENERALI
    // =========================================================
    double rate_hz;
    pnh.param("rate_hz", rate_hz, 200.0);

    double pose_scale;
    pnh.param("pose_scale", pose_scale, 0.001); // se pose in mm -> m

    std::string default_frame_id;
    pnh.param<std::string>("default_frame_id", default_frame_id, "world");

    int log_queue;
    pnh.param("log_queue", log_queue, 500);

    // =========================================================
    // TRAIETTORIA SINUSOIDALE (lungo Y del punto traslato)
    // =========================================================
    double freq_hz, amplitude_m;
    pnh.param("freq_hz", freq_hz, 0.25);
    pnh.param("amplitude_m", amplitude_m, 0.005);

    // =========================================================
    // OFFSET DEL PUNTO CONTROLLATO RISPETTO ALL'HIP
    // =========================================================
    double offset_x_m, offset_y_m, offset_z_m;
    pnh.param("offset_x_m", offset_x_m, 0.0);
    pnh.param("offset_y_m", offset_y_m, 0.0);
    pnh.param("offset_z_m", offset_z_m, 0.0);

    // =========================================================
    // CONTROLLO ASSE PRINCIPALE Y
    // =========================================================
    double Kp_y, Kd_y, Ki_y, Fy_max, K_ff, Ka_y;
    pnh.param("Kp_y", Kp_y, 600.0);
    pnh.param("Kd_y", Kd_y, 10.0);
    pnh.param("Ka_y", Ka_y, 35.0);
    pnh.param("Ki_y", Ki_y, 0.0);
    pnh.param("K_ff", K_ff, 15.0);
    pnh.param("Fy_max", Fy_max, 25.0);

    // =========================================================
    // CONTENIMENTO FORTE SU X E Z
    // =========================================================
    double Kp_xz, Kd_xz, Fx_max, Fz_max;
    double Iy_max;
    pnh.param("Kp_xz", Kp_xz, 450.0);
    pnh.param("Kd_xz", Kd_xz, 25.0);
    pnh.param("Fx_max", Fx_max, 25.0);
    pnh.param("Fz_max", Fz_max, 25.0);
    pnh.param("Iy_max", Iy_max, 1.0);

    // opzionale: limite complessivo del modulo forza
    double F_total_max;
    pnh.param("F_total_max", F_total_max, 5.0);

    // =========================================================
    // FILTRI
    // =========================================================
    double tau_p, tau_v, tau_F;
    pnh.param("tau_p", tau_p, 0.0);
    pnh.param("tau_v", tau_v, 0.001);
    pnh.param("tau_F", tau_F, 0.02);

    // deadband asse principale
    double deadband_y_m;
    pnh.param("deadband_y_m", deadband_y_m, 0.0);

    // =========================================================
    // ALLINEAMENTO INIZIALE
    // =========================================================
    double start_align_s;
    pnh.param("start_align_s", start_align_s, 0.5);

    // =========================================================
    // ROS I/O
    // =========================================================
    ros::Subscriber sub_pose = nh.subscribe("pose", 10, poseCallback);

    ROS_INFO("breathing_lock: attesa publisher su /pose ...");
    while (ros::ok() && sub_pose.getNumPublishers() == 0)
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        ROS_INFO_THROTTLE(1.0, "breathing_lock: nessun publisher su /pose, attendo...");
    }
    ROS_INFO("breathing_lock: publisher trovato! Attesa primo messaggio...");

    ros::Rate wait_rate(200);
    while (ros::ok() && !g_have_pose)
    {
        ros::spinOnce();
        wait_rate.sleep();
    }
    ROS_INFO("breathing_lock: prima pose ricevuta, avvio controllo.");

    ros::Publisher pub_feedback = nh.advertise<geomagic_control::DeviceFeedback>("force_feedback", 1);
    ros::Publisher pub_hip      = nh.advertise<geometry_msgs::PointStamped>("HIP", log_queue);
    ros::Publisher pub_ctrl     = nh.advertise<geometry_msgs::PointStamped>("CTRL_point", log_queue);
    ros::Publisher pub_ctrl_des = nh.advertise<geometry_msgs::PointStamped>("CTRL_point_des", log_queue);

    ROS_INFO("breathing_lock: attesa prima pose su /pose ...");
    ros::Time t_wait_start = ros::Time::now();

    while (ros::ok() && !g_have_pose)
    {
        ros::spinOnce();
        wait_rate.sleep();
        ROS_INFO_THROTTLE(1.0, "breathing_lock: ancora in attesa di /pose ... (%.0f s)",
                          (ros::Time::now() - t_wait_start).toSec());
    }

    ROS_INFO("breathing_lock: prima pose ricevuta, avvio controllo.");

    // HIP iniziale in metri
    double hip_x = g_pose.pose.position.x * pose_scale;
    double hip_y = g_pose.pose.position.y * pose_scale;
    double hip_z = g_pose.pose.position.z * pose_scale;

    // Punto controllato iniziale = HIP + offset
    double ctrl_x0 = hip_x + offset_x_m;
    double ctrl_y0 = hip_y + offset_y_m;
    double ctrl_z0 = hip_z + offset_z_m;

    ROS_INFO("Punto controllato iniziale (m): [%f, %f, %f]", ctrl_x0, ctrl_y0, ctrl_z0);
    ROS_INFO("Offset (m): [%f, %f, %f]", offset_x_m, offset_y_m, offset_z_m);
    ROS_INFO("Traiettoria: asse y, amp=%f m, freq=%f Hz", amplitude_m, freq_hz);
    ROS_INFO("Guida asse y: Kp_y=%f Kd_y=%f Fy_max=%f", Kp_y, Kd_y, Fy_max);
    ROS_INFO("Contenimento xz forte: Kp_xz=%f Kd_xz=%f Fx_max=%f Fz_max=%f",
             Kp_xz, Kd_xz, Fx_max, Fz_max);
    ROS_INFO("Filtri: tau_p=%f tau_v=%f tau_F=%f", tau_p, tau_v, tau_F);

    // =========================================================
    // AVVIO THREAD TASTIERA
    // =========================================================
    std::thread kb_thread(keyboardThread);
    kb_thread.detach();

    ros::Rate loop_rate(rate_hz);

    ros::Time t_prev = ros::Time::now();
    ros::Time t0 = ros::Time::now();  // verrà reimpostato quando parte la sinusoide

    // stati filtrati del punto controllato
    double ctrl_x_f = ctrl_x0;
    double ctrl_y_f = ctrl_y0;
    double ctrl_z_f = ctrl_z0;

    double ctrl_x_prev = ctrl_x_f;
    double ctrl_y_prev = ctrl_y_f;
    double ctrl_z_prev = ctrl_z_f;

    double vx_f = 0.0;
    double vy_f = 0.0;
    double vz_f = 0.0;

    double Fx_f = 0.0;
    double Fy_f = 0.0;
    double Fz_f = 0.0;
    double Iy = 0.0;

    // fase di allineamento
    bool aligned = false;
    ros::Time t_align0 = ros::Time::now();

    while (ros::ok())
    {
        ros::spinOnce();

        const ros::Time t_now = ros::Time::now();
        const double dt = std::max((t_now - t_prev).toSec(), 1e-6);
        t_prev = t_now;

        const double alpha_p = dt / (tau_p + dt);
        const double alpha_v = dt / (tau_v + dt);
        const double alpha_F = dt / (tau_F + dt);

        // =====================================================
        // HIP attuale
        // =====================================================
        hip_x = g_pose.pose.position.x * pose_scale;
        hip_y = g_pose.pose.position.y * pose_scale;
        hip_z = g_pose.pose.position.z * pose_scale;

        const double ctrl_x = hip_x + offset_x_m;
        const double ctrl_y = hip_y + offset_y_m;
        const double ctrl_z = hip_z + offset_z_m;

        ctrl_x_f = lpf(ctrl_x_f, ctrl_x, alpha_p);
        ctrl_y_f = lpf(ctrl_y_f, ctrl_y, alpha_p);
        ctrl_z_f = lpf(ctrl_z_f, ctrl_z, alpha_p);

        const double vx = (ctrl_x_f - ctrl_x_prev) / dt;
        const double vy = (ctrl_y_f - ctrl_y_prev) / dt;
        const double vz = (ctrl_z_f - ctrl_z_prev) / dt;

        ctrl_x_prev = ctrl_x_f;
        ctrl_y_prev = ctrl_y_f;
        ctrl_z_prev = ctrl_z_f;

        vx_f = lpf(vx_f, vx, alpha_v);
        vy_f = lpf(vy_f, vy, alpha_v);
        vz_f = lpf(vz_f, vz, alpha_v);

        // =====================================================
        // LOG HIP e punto controllato
        // =====================================================
        const std::string fid = g_pose.header.frame_id.empty() ? default_frame_id : g_pose.header.frame_id;

        geometry_msgs::PointStamped hip_msg, ctrl_msg, ctrl_des_msg;
        hip_msg.header.stamp    = t_now;
        hip_msg.header.frame_id = fid;
        ctrl_msg.header.stamp    = t_now;
        ctrl_msg.header.frame_id = fid;
        ctrl_des_msg.header.stamp    = t_now;
        ctrl_des_msg.header.frame_id = fid;

        hip_msg.point.x = hip_x;
        hip_msg.point.y = hip_y;
        hip_msg.point.z = hip_z;

        ctrl_msg.point.x = ctrl_x_f - ctrl_x0;
        ctrl_msg.point.y = ctrl_y_f - ctrl_y0;
        ctrl_msg.point.z = ctrl_z_f - ctrl_z0;

        // =====================================================
        // ALLINEAMENTO INIZIALE
        // =====================================================
        if (!aligned)
        {
            geomagic_control::DeviceFeedback fb0;
            fb0.force.x = 0.0;
            fb0.force.y = 0.0;
            fb0.force.z = 0.0;
            fb0.lock = std::vector<uint8_t>(3, 0);
            fb0.position.x = 0.0;
            fb0.position.y = 0.0;
            fb0.position.z = 0.0;
            pub_feedback.publish(fb0);

            ctrl_des_msg.point.x = ctrl_x0;
            ctrl_des_msg.point.y = ctrl_y0;
            ctrl_des_msg.point.z = ctrl_z0;

            pub_hip.publish(hip_msg);
            pub_ctrl.publish(ctrl_msg);
            pub_ctrl_des.publish(ctrl_des_msg);

            if ((t_now - t_align0).toSec() >= start_align_s)
            {
                ctrl_x0 = ctrl_x_f;
                ctrl_y0 = ctrl_y_f;
                ctrl_z0 = ctrl_z_f;

                ctrl_x_prev = ctrl_x_f;
                ctrl_y_prev = ctrl_y_f;
                ctrl_z_prev = ctrl_z_f;

                vx_f = 0.0;
                vy_f = 0.0;
                vz_f = 0.0;

                Fx_f = 0.0;
                Fy_f = 0.0;
                Fz_f = 0.0;
                Iy = 0.0;

                aligned = true;
                ROS_INFO("Allineamento completato. Centro agganciato (punto controllato) a [%f, %f, %f]",
                         ctrl_x0, ctrl_y0, ctrl_z0);
            }

            loop_rate.sleep();
            continue;
        }

        // =====================================================
        // ATTESA INVIO DA TASTIERA: finché non premuto, forza zero
        // =====================================================
        if (!g_sine_started.load())
        {
            geomagic_control::DeviceFeedback fb_wait;
            fb_wait.force.x = 0.0;
            fb_wait.force.y = 0.0;
            fb_wait.force.z = 0.0;
            fb_wait.lock = std::vector<uint8_t>(3, 0);
            fb_wait.position.x = 0.0;
            fb_wait.position.y = 0.0;
            fb_wait.position.z = 0.0;
            pub_feedback.publish(fb_wait);

            ctrl_des_msg.point.x = 0.0;
            ctrl_des_msg.point.y = 0.0;
            ctrl_des_msg.point.z = 0.0;

            pub_hip.publish(hip_msg);
            pub_ctrl.publish(ctrl_msg);
            pub_ctrl_des.publish(ctrl_des_msg);

            // Aggiorna il centro al punto corrente (aggancio continuo in attesa)
            ctrl_x0 = ctrl_x_f;
            ctrl_y0 = ctrl_y_f;
            ctrl_z0 = ctrl_z_f;

            loop_rate.sleep();
            continue;
        }

        // Prima iterazione dopo INVIO: inizia la sinusoide dal tempo corrente
        // Usa una variabile statica per rilevare il primo ciclo attivo
        static bool sine_just_started = true;
        if (sine_just_started)
        {
            // Agganciamo il centro all'ultima posizione filtrata
            ctrl_x0 = ctrl_x_f;
            ctrl_y0 = ctrl_y_f;
            ctrl_z0 = ctrl_z_f;

            // Reset stati
            ctrl_x_prev = ctrl_x_f;
            ctrl_y_prev = ctrl_y_f;
            ctrl_z_prev = ctrl_z_f;

            vx_f = 0.0; vy_f = 0.0; vz_f = 0.0;
            Fx_f = 0.0; Fy_f = 0.0; Fz_f = 0.0;
            Iy = 0.0;

            // La sinusoide parte da zero fase
            t0 = t_now;
            sine_just_started = false;
            ROS_INFO("Sinusoide avviata. Centro agganciato a [%f, %f, %f]",
                     ctrl_x0, ctrl_y0, ctrl_z0);
        }

        // =====================================================
        // TRAIETTORIA DESIDERATA DEL PUNTO CONTROLLATO
        // =====================================================
        const double tt = (t_now - t0).toSec();
        const double w = 2.0 * M_PI * freq_hz;

        const double s      =  amplitude_m * std::sin(w * tt);
        const double s_dot  =  amplitude_m * w * std::cos(w * tt);
        const double s_ddot = -amplitude_m * w * w * std::sin(w * tt);

        const double ctrl_x_des = ctrl_x0;
        const double ctrl_y_des = ctrl_y0 + s;
        const double ctrl_z_des = ctrl_z0;

        ctrl_des_msg.point.x = 0.0;
        ctrl_des_msg.point.y = s;
        ctrl_des_msg.point.z = 0.0;

        pub_hip.publish(hip_msg);
        pub_ctrl.publish(ctrl_msg);
        pub_ctrl_des.publish(ctrl_des_msg);

        // =====================================================
        // ERRORI
        // =====================================================
        double e_y = ctrl_y_des - ctrl_y_f;
        if (std::abs(e_y) < deadband_y_m) e_y = 0.0;

        const double e_vy = s_dot - vy_f;

        Iy += e_y * dt;
        Iy = clamp(Iy, -1.0, 1.0);

        const double e_x = ctrl_x_des - ctrl_x_f;
        const double e_z = ctrl_z_des - ctrl_z_f;

        ROS_INFO_THROTTLE(0.5,
            "[breathing] e_y=%.4f m  e_vy=%.4f m/s  e_x=%.4f m  e_z=%.4f m  "
            "Fy=%.2f N  |s|=%.4f m  |s_dot|=%.4f m/s",
            e_y, e_vy, e_x, e_z,
            Kp_y * e_y + Kd_y * e_vy,
            std::abs(s), std::abs(s_dot));

        // =====================================================
        // LEGGI DI FORZA
        // =====================================================
        double Fy = Kp_y * e_y + Kd_y * e_vy + Ki_y * Iy + s_dot * K_ff + Ka_y * s_ddot;
        double Fx = Kp_xz * e_x - Kd_xz * vx_f;
        double Fz = Kp_xz * e_z - Kd_xz * vz_f;

        Fx = clamp(Fx, -Fx_max, Fx_max);
        Fy = clamp(Fy, -Fy_max, Fy_max);
        Fz = clamp(Fz, -Fz_max, Fz_max);

        double Fnorm = std::sqrt(Fx*Fx + Fy*Fy + Fz*Fz);
        if (Fnorm > F_total_max && Fnorm > 1e-9)
        {
            const double sc = F_total_max / Fnorm;
            Fx *= sc;
            Fy *= sc;
            Fz *= sc;
        }

        Fx_f = lpf(Fx_f, Fx, alpha_F);
        Fy_f = lpf(Fy_f, Fy, alpha_F);
        Fz_f = lpf(Fz_f, Fz, alpha_F);

        // =====================================================
        // PUBLISH FORCE FEEDBACK
        // =====================================================
        geomagic_control::DeviceFeedback fb;
        fb.force.x = Fx_f;
        fb.force.y = Fy_f;
        fb.force.z = Fz_f;
        fb.lock = std::vector<uint8_t>(3, 0);
        fb.position.x = 0.0;
        fb.position.y = 0.0;
        fb.position.z = 0.0;
        pub_feedback.publish(fb);

        loop_rate.sleep();
    }

    return 0;
}
