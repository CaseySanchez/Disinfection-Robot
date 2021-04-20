#include "core_states.hpp"

void IdleState::enter()
{
}

void IdleState::exit()
{
}

std::optional<std::string> IdleState::update()
{
    return { };
}

void ManualState::enter()
{
    wiringPiSetup();

    pinMode(26, OUTPUT);
    pinMode(27, OUTPUT);
    pinMode(28, OUTPUT);
    pinMode(29, OUTPUT);

    softPwmCreate(21, 0, 100);
    softPwmCreate(22, 0, 100);
    softPwmCreate(23, 0, 100);
    softPwmCreate(24, 0, 100);

    m_server.set_open_handler(websocketpp::lib::bind(&ManualState::openHandler, this, websocketpp::lib::placeholders::_1));
    m_server.set_close_handler(websocketpp::lib::bind(&ManualState::closeHandler, this, websocketpp::lib::placeholders::_1));
    m_server.set_message_handler(websocketpp::lib::bind(&ManualState::messageHandler, this, websocketpp::lib::placeholders::_1, websocketpp::lib::placeholders::_2));
    
    m_server.set_access_channels(websocketpp::log::alevel::all);
    m_server.set_error_channels(websocketpp::log::elevel::all);

    m_server.init_asio();
    m_server.listen(9002);
    m_server.start_accept();

    m_thread = std::thread(&ManualState::serverThread, this);
}

void ManualState::exit()
{
    m_server.stop_listening();

    m_thread.join();
}

std::optional<std::string> ManualState::update()
{
    std::lock_guard<std::mutex> lock_guard(m_mutex);

    auto lerp = [](float const &a, float const &b, float const &t) -> float {
        return a * (t - 1.0f) + b * t;
    }

    m_motor_speed[0] = lerp(m_motor_speed[0], m_target_speed[0], 0.1f);
    m_motor_speed[1] = lerp(m_motor_speed[1], m_target_speed[1], 0.1f);
    m_motor_speed[2] = lerp(m_motor_speed[2], m_target_speed[2], 0.1f);
    m_motor_speed[3] = lerp(m_motor_speed[3], m_target_speed[3], 0.1f);

    digitalWrite(26, m_motor_speed[0] > 0.0 ? 0 : 1);
    digitalWrite(27, m_motor_speed[1] > 0.0 ? 0 : 1);
    digitalWrite(28, m_motor_speed[2] > 0.0 ? 0 : 1);
    digitalWrite(29, m_motor_speed[3] > 0.0 ? 0 : 1);

    std::array<int32_t, 4> duty_cycle = {
        static_cast<int32_t>(std::abs(m_motor_speed[0]) * 100.0f),
        static_cast<int32_t>(std::abs(m_motor_speed[1]) * 100.0f),
        static_cast<int32_t>(std::abs(m_motor_speed[2]) * 100.0f),
        static_cast<int32_t>(std::abs(m_motor_speed[3]) * 100.0f)
    };

    softPwmWrite(21, duty_cycle[0]);
    softPwmWrite(22, duty_cycle[1]);
    softPwmWrite(23, duty_cycle[2]);
    softPwmWrite(24, duty_cycle[3]);

    return { };
}

void ManualState::serverThread()
{
    m_server.run();
}

void ManualState::openHandler(websocketpp::connection_hdl connection_hdl)
{
}

void ManualState::closeHandler(websocketpp::connection_hdl connection_hdl)
{
}

void ManualState::messageHandler(websocketpp::connection_hdl connection_hdl, websocketpp::server<websocketpp::config::asio>::message_ptr message) 
{
    std::lock_guard<std::mutex> lock_guard(m_mutex);

    auto json = nlohmann::json::parse(message->get_payload());

    m_target_speed[0] = json["motor_0"];
    m_target_speed[1] = json["motor_1"];
    m_target_speed[2] = json["motor_2"];
    m_target_speed[3] = json["motor_3"];

    std::cout << message->get_payload() << std::endl;
}

void AutoState::enter()
{
    ROS_INFO("start");
}

void AutoState::exit()
{
    ROS_INFO("stop");
}

std::optional<std::string> AutoState::update()
{
    ROS_INFO("update");

    return { };
}
