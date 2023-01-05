#include "port/CanPort.hpp"

constexpr double TIMEOUT = 2;

static const uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static const uint8_t len2dlc[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8,      // 0 - 8
    9, 9, 9, 9,                     // 9 - 12
    10, 10, 10, 10,                 // 13 - 16
    11, 11, 11, 11,                 // 17 - 20
    12, 12, 12, 12,                 // 21 - 24
    13, 13, 13, 13, 13, 13, 13, 13, // 25 - 32
    14, 14, 14, 14, 14, 14, 14, 14, // 33 - 40
    14, 14, 14, 14, 14, 14, 14, 14, // 41 - 48
    15, 15, 15, 15, 15, 15, 15, 15, // 49 - 56
    15, 15, 15, 15, 15, 15, 15, 15, // 57 - 64
};

/**
 * @brief 根据设备名选择通信设备并启动读写线程
 *
 * @param port_name
 */
wmj::CanPort::CanPort(std::string port_name)
{
    //可以使用can设备的标志位
    this->canUseThisPort = true;

    // create a socketfd
    if ((m_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        std::cout << "[WMJERROR]Code=04 U2CAN can't open" << port_name << std::endl;
        std::cerr << "Create Socket Error!!" << std::endl;
        canUseThisPort = false;
    }

    // socket和设备进行绑定
    strcpy(m_ifr.ifr_name, port_name.c_str());
    m_ifr.ifr_ifindex = if_nametoindex(m_ifr.ifr_name);

    if (ioctl(m_sock, SIOCGIFINDEX, &m_ifr) < 0)
    {
        std::cout << "[WMJERROR]Code=04 U2CAN can't open" << port_name << std::endl;
        std::cerr << "Set io error" << std::endl;
        canUseThisPort = false;
    }

    // bind socket
    m_addr.can_family = AF_CAN;
    m_addr.can_ifindex = m_ifr.ifr_ifindex;
    if (bind(m_sock, (sockaddr *)&m_addr, sizeof(m_addr)) < 0)
    {
        std::cout << "[WMJERROR]Code=04 U2CAN can't open" << port_name << std::endl;
        std::cerr << "Bind Error!!" << std::endl;
        canUseThisPort = false;
    }

    //
    int fdflags = fcntl(m_sock, F_GETFL, 0);
    if (fcntl(m_sock, F_SETFL, fdflags | O_NONBLOCK) < 0)
    {
        std::cout << "[WMJERROR]Code=04 U2CAN can't open" << port_name << std::endl;
        std::cerr << "Set Nonblock Error!!" << std::endl;
        canUseThisPort = false;
    }
    if (canUseThisPort)
    {
        std::cout << port_name << std::endl;
        this->m_read_gimbal_buffer.resize(255);
        this->m_read_chassis_buffer.resize(255);
        this->m_read_shooter_buffer.resize(255);
        this->m_read_maincontrol_buffer.resize(255);
        this->m_read_vision_buffer.resize(255);
        this->m_read_judge_buffer.resize(255);
        this->m_read_communicate_buffer.resize(255);
        this->m_read_gyro_buffer.resize(255);
        this->m_read_state_buffer.resize(255);
        // std::cerr << "Prepare to Start the thread" << std::endl ;
        this->startThread();
    }
}

/**
 * @brief 启动读写线程
 */
void wmj::CanPort::startThread()
{
    m_thread_run = true;
    this->m_readThread = std::thread(&wmj::CanPort::readRun, this);
    this->m_readThread.detach();
    this->m_writeThread = std::thread(&wmj::CanPort::writeRun, this);
    this->m_writeThread.detach();
}

void wmj::CanPort::stopThread()
{
    m_thread_run = false;
}
/**
 * @brief 由于类成员中有不可拷贝的对象，所以覆盖掉默认的拷贝构造函数，以解决兼容性问题。
 *
 * @param tp
 */
wmj::CanPort::CanPort(const CanPort &tp)
{
    m_addr = tp.m_addr;
    m_ifr = tp.m_ifr;
}

wmj::CanPort::~CanPort()
{
    this->stopThread();
}

/**
 * @brief 发送一帧数据（外部接口，由于是非阻塞的，所以）
 *
 * @param data
 *
 * @return 发送是否成功
 */
bool wmj::CanPort::sendFrame(Buffer &data)
{
    m_write_buffer_mutex.lock();
    try
    {
        m_write_buffer.push(data);
    }
    catch (std::exception &ex)
    {
        std::cerr << ex.what();
        m_write_buffer_mutex.unlock();
        return false;
    }
    if (m_write_buffer.size() > 5)
    {
        m_write_buffer.pop();
    }
    m_write_buffer_mutex.unlock();
    return true;
}
/*
#################发送数据，写线程函数
*/
bool wmj::CanPort::tryWrite()
{
    std::lock_guard<std::mutex> lock_guard_can(m_can_mutex);
    std::lock_guard<std::mutex> lock_guard_write(m_write_buffer_mutex);
    int required_mtu{};
    if (!m_write_buffer.empty())
    {
        required_mtu = parseData(m_write_buffer.front(), this->m_send_frame); // parse data to can frame
        m_write_buffer.pop();
    }
    else
    {
        m_can_mutex.unlock();
        m_write_buffer_mutex.unlock();
        return false;
    }

    if (send(m_sock, &m_send_frame, required_mtu, MSG_DONTWAIT) != required_mtu)
    {
        // std::cerr << "Error Write" << std::endl;
        // std::cout << required_mtu << std::endl;
        // std::cout << write( _sock, &_send_frame, required_mtu );
        // std::cout << "Error Write And errorno is " << errno << std::endl;
        // std::cout << strerror(errno) << std::endl;
        if (errno == 100)
        {
            pid_t status;
            status = system("cd /home/nvidia && ./enCanBus.sh");
            if (!WIFEXITED(status))
            {
                std::cout << "[WMJERROR]Code=05 U2CAN can't open" << std::endl;

                std::cout << "write error" << std::endl;
                canUseThisPort = false;
            }

            if (WEXITSTATUS(status) != 0)
            {
                std::cout << "[WMJERROR]Code=05 U2CAN can't open" << std::endl;
                std::cout << "write error" << std::endl;
                HINT
                    canUseThisPort = false;
            }
        }
        return false;
    }
    return true;
}
/**
 * @brief 尝试读取系统缓冲区
 *
 * @return
 */
bool wmj::CanPort::tryRead()
{
    std::lock_guard<std::mutex> lock_guard_can(m_can_mutex);
    clock_t clock_begin = clock();
    bool flag = true;
    while (flag && canUseThisPort)
    {
        int nbytes = recv(m_sock, &(m_read_frame), sizeof(m_read_frame), MSG_DONTWAIT);
        // std::cout << nbytes << std::endl;
        if (nbytes < 0)
        {
            m_start_flag = m_start_flag + 1;
            if (m_start_flag > 50)
            {
                std::cout << "[WMJERROR]Code=06 U2CAN can't open" << m_ifr.ifr_name << std::endl;
                std::cout << "read error" << std::endl;
                canUseThisPort = false;
            }
            if (errno == 11)
            {
                // std::cout << strerror(errno) << std::endl;
                ;
            }
            if (errno == EAGAIN)
            {
                // std::cerr << _warning("EAGAIN no more data in buffer") << std::endl;
                ;
            }
            flag = false;
        }
        else if (nbytes == CAN_MTU)
        {
            m_start_flag = 0;
            std::lock_guard<std::mutex> lock_guard_read(m_read_buffer_mutex);
            parseCanFrame(m_read_buffer, m_read_frame);
            switch (m_read_buffer[0])
            {
            case Gimbal:
                std::copy(m_read_buffer.begin() + 1, m_read_buffer.end(), m_read_gimbal_buffer.begin());
                break;
            case Gyro:
                std::copy(m_read_buffer.begin() + 1, m_read_buffer.end(), m_read_gyro_buffer.begin());
                break;
            case MainControl:
                std::copy(m_read_buffer.begin() + 1, m_read_buffer.end(), m_read_maincontrol_buffer.begin());
                break;
            case Judge:
                std::copy(m_read_buffer.begin() + 1, m_read_buffer.end(), m_read_judge_buffer.begin());
                break;
            case ComRecv:
                std::copy(m_read_buffer.begin() + 1, m_read_buffer.end(), m_read_communicate_buffer.begin());
                break;
            case Chassis:
                std::copy(m_read_buffer.begin() + 1, m_read_buffer.end(), m_read_chassis_buffer.begin());
                break;
            case Shooter:
                std::copy(m_read_buffer.begin() + 1, m_read_buffer.end(), m_read_shooter_buffer.begin());
                break;
            case Vision:
                std::copy(m_read_buffer.begin() + 1, m_read_buffer.end(), m_read_vision_buffer.begin());
                break;
            case State:
                std::copy(m_read_buffer.begin() + 1, m_read_buffer.end(), m_read_state_buffer.begin());
                break;
            default:
                break;
            }
            break;
        }
        else
        {
            std::cerr << "Incomplete data packs" << std::endl;
        }

        if ((double)(clock() - clock_begin) / CLOCKS_PER_SEC > (double)TIMEOUT / 1000.0)
        {
            // std::cerr << "TimeOut!" << std::endl;//测试硬触发，暂时测试，删除报错
            flag = false;
        }
    }
    return flag;
}

/**
 * @brief 读线程--1ms间隔循环读取底层缓冲区
 */
void wmj::CanPort::readRun()
{
    std::cout << "read thread start !" << std::endl;
    while (m_thread_run)
    {
        this->tryRead();
        usleep(500);
    }
    std::cout << "read thread stop !" << std::endl;
    return;
}

/**
 * @brief 写线程--1ms间隔将任务队列中的消息发出
 */
void wmj::CanPort::writeRun()
{
    std::cout << "write thread start !" << std::endl;
    while (m_thread_run)
    {
        this->tryWrite();
        usleep(200);
    }
    std::cout << "write thread stop !" << std::endl;
    return;
}

uint8_t wmj::CanPort::can_dlc2len(uint8_t can_dlc)
{
    return dlc2len[can_dlc & 0x0f];
}

uint8_t wmj::CanPort::can_len2dlc(uint8_t len)
{
    if (len > 64)
        return 0xf;
    return len2dlc[len];
}

uint8_t wmj::CanPort::asc2nibble(uint8_t data)
{
    if (data >= '0' && data <= '9')
        return data - '0';
    if (data >= 'A' && data <= 'F')
        return data - 'A' + 10;
    if (data >= 'a' && data <= 'f')
        return data - 'a' + 10;
    else
        return 16;
}

int wmj::CanPort::parseData(Buffer &data, canfd_frame &frame)
{
    int idx, len;
    int ret = CAN_MTU;

    len = data.size();
    memset(&frame, 0, sizeof(frame));

    if (len < 2)
        return 0;

    idx = 1;
    switch (data[0])
    {
        //Gimbal_down
    case Gimbal:
        frame.can_id = 0x311;
        break;
    case Shooter:
        frame.can_id = 0x321;
        break;
    case Chassis:
        frame.can_id = 0x301;
        break;
    case MainControl:
        frame.can_id = 0x354;
        break;
    case Draw:
        frame.can_id = 0x348;
        break;
    case ComSend:
        frame.can_id = 0x340;
        break;
    case Vision:
        frame.can_id = 0x354;
        break;
    default:
        break;
    }

    for (int i = 0; i < len - 1; i++)
    {
        frame.data[i] = data[idx++];
    }
    frame.len = len - 1;
    return ret;
}

int wmj::CanPort::parseCanFrame(Buffer &data, canfd_frame &frame)
{
    
    // char buffer[33];
    // sprintf(buffer,"%x",frame.can_id);
    // std::cout<<buffer<<std::endl;
    data.clear();
    switch (frame.can_id)
    {
    case 0x314:
        data.push_back(Gimbal);
        break;
    case 0x316:
        data.push_back(Gyro);
        break;
    case 0x304:
        data.push_back(Chassis);
        break;
    case 0x334:
        data.push_back(MainControl);
        break;
    case 0x344:
        data.push_back(Judge);
        break;
    case 0x342:
        data.push_back(ComRecv);
        break;
    case 0x345:
        data.push_back(State);
        break;
    default:
        data.push_back(NONE);
    }
    for (auto c : frame.data)
    {
        data.push_back(c);
    }
    return data.size();
}

/**
 * @brief 读取一帧数据（外部接口） Judge回传清空data1和data2读取实时弹速
 *
 * @param flag 要读取的数据帧类型
 *
 * @return 原始数据帧
 */
wmj::Buffer wmj::CanPort::readFrame(int flag)
{
    wmj::Buffer value;
    std::lock_guard<std::mutex> lock_guard(m_read_buffer_mutex);
    switch (flag)
    {
    case NONE:
        value = Buffer{0};
        break;
    case Gimbal:
        value = m_read_gimbal_buffer;
        break;
    case Shooter:
        value = m_read_shooter_buffer;
        return value;
    case Chassis:
        value = m_read_chassis_buffer;
        return value;
    case MainControl:
        value = m_read_maincontrol_buffer;
        break;
    case Judge:
        value = m_read_judge_buffer;
        if(m_clear_shoot_speed)
        {
            m_read_judge_buffer[1] = 0;
            m_read_judge_buffer[2] = 0;
            m_clear_shoot_speed = false;
        }
        break;
    case Gyro:
        value = m_read_gyro_buffer;
        break;
    case Vision:
        value = m_read_vision_buffer;
        break;
    case State:
        value = m_read_state_buffer;
        break;
    default:
        value = Buffer{0xff};
        break;
    }
    return value;
}
