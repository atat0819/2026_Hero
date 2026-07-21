#include "RMRefereeSystem.hpp"
#include "RMRefereeSystemCRC.hpp"

/*======================================================================*/
/*                        全局变量的初始化                               */
/*======================================================================*/

RMRefereeSystemData_t            RMRefereeSystemData           = { 0 };  // 裁判系统当前帧数据

unsigned char                    MyRefereeSys8Data              = 0;      // 裁判系统串口数据接收缓冲区（单字节）

/* ===================== 0x0001~0x0003 比赛相关 ===================== */
game_status_t                    game_status_0x0001             = { 0 };  // 0x0001 比赛状态数据
game_result_t                    game_result_0x0002             = { 0 };  // 0x0002 比赛结果数据
game_robot_HP_t                  game_robot_HP_0x0003           = { 0 };  // 0x0003 机器人血量数据

/* ===================== 0x0101~0x0105 场地 / 判罚 / 飞镖 ===================== */
event_data_t                     event_data_0x0101              = { 0 };  // 0x0101 场地事件数据
referee_warning_t                referee_warning_0x0104         = { 0 };  // 0x0104 裁判警告数据
dart_info_t                      dart_info_0x0105               = { 0 };  // 0x0105 飞镖发射相关数据

/* ===================== 0x0201~0x020E 机器人状态 ===================== */
robot_status_t                   robot_status_0x0201            = { 0 };  // 0x0201 机器人性能体系数据
power_heat_data_t                power_heat_data_0x0202         = { 0 };  // 0x0202 实时底盘缓冲能量和枪口热量
robot_pos_t                      robot_pos_0x0203               = { 0 };  // 0x0203 机器人位置数据
buff_t                           buff_0x0204                    = { 0 };  // 0x0204 机器人增益和能量数据
hurt_data_t                      hurt_data_0x0206               = { 0 };  // 0x0206 伤害状态数据
shoot_data_t                     shoot_data_0x0207              = { 0 };  // 0x0207 实时射击数据
projectile_allowance_t           projectile_allowance_0x0208    = { 0 };  // 0x0208 允许发弹量
rfid_status_t                    rfid_status_0x0209             = { 0 };  // 0x0209 RFID 模块状态
dart_client_cmd_t                dart_client_cmd_0x020A         = { 0 };  // 0x020A 飞镖选手端指令数据
ground_robot_position_t          ground_robot_position_0x020B   = { 0 };  // 0x020B 地面机器人位置数据
radar_mark_data_t                radar_mark_data_0x020C         = { 0 };  // 0x020C 雷达标记进度数据
sentry_info_t                    sentry_info_0x020D             = { 0 };  // 0x020D 哨兵自主决策信息同步
radar_info_t                     radar_info_0x020E              = { 0 };  // 0x020E 雷达自主决策信息同步


/*======================================================================*/
/*                          函数的声明                                  */
/*======================================================================*/

static void RMRefereeSystemParseData(uint8_t *MypDatas, uint16_t size);
static void RMRefereeSystemGetData(uint8_t MypData);
void RMRefereeSystemParse(void);


/************************************************************************
 * @brief  解析裁判系统数据（一帧完整数据）
 * @param  MypDatas  指向帧头起始位置的指针
 * @param  size      帧总长度
 ************************************************************************/
static void RMRefereeSystemParseData(uint8_t *MypDatas, uint16_t size)
{
    if (size < 9)  // 最小帧：帧头(5) + cmd_id(2) + CRC16(2)
    {
        return;
    }

    /* ============ 解析帧头字段 ============ */
    RMRefereeSystemData.SOF         = MypDatas[0];
    RMRefereeSystemData.data_length = (uint16_t)(MypDatas[1] | (MypDatas[2] << 8));
    RMRefereeSystemData.seq         = MypDatas[3];
    RMRefereeSystemData.CRC8        = MypDatas[4];
    RMRefereeSystemData.cmd_id      = (uint16_t)(MypDatas[5] | (MypDatas[6] << 8));

    /* ============ 拷贝数据段 ============ */
    if (RMRefereeSystemData.data_length > sizeof(RMRefereeSystemData.data))
    {
        return;  // 数据段超出缓冲区，丢弃此帧
    }

    for (uint16_t i = 0; i < RMRefereeSystemData.data_length; i++)
    {
        RMRefereeSystemData.data[i] = MypDatas[7 + i];
    }

    /* ============ 解析帧尾 CRC16 ============ */
    uint16_t crc_index = (uint16_t)(7 + RMRefereeSystemData.data_length);
    if (crc_index + 1 >= size)
    {
        return;
    }
    RMRefereeSystemData.frame_tail = (uint16_t)(MypDatas[crc_index] | (MypDatas[crc_index + 1] << 8));

    /* ============ 根据 cmd_id 分发到对应全局变量 ============ */
    switch (RMRefereeSystemData.cmd_id)
    {
    /* ---- 比赛相关 ---- */
    case RM_CMD_GAME_STATUS:           // 0x0001
        memcpy(&game_status_0x0001, (void *)RMRefereeSystemData.data, sizeof(game_status_0x0001));
        break;
    case RM_CMD_GAME_RESULT:           // 0x0002
        memcpy(&game_result_0x0002, (void *)RMRefereeSystemData.data, sizeof(game_result_0x0002));
        break;
    case RM_CMD_GAME_ROBOT_HP:         // 0x0003
        memcpy(&game_robot_HP_0x0003, (void *)RMRefereeSystemData.data, sizeof(game_robot_HP_0x0003));
        break;

    /* ---- 场地 / 判罚 / 飞镖 ---- */
    case RM_CMD_EVENT_DATA:            // 0x0101
        memcpy(&event_data_0x0101, (void *)RMRefereeSystemData.data, sizeof(event_data_0x0101));
        break;
    case RM_CMD_REFEREE_WARNING:       // 0x0104
        memcpy(&referee_warning_0x0104, (void *)RMRefereeSystemData.data, sizeof(referee_warning_0x0104));
        break;
    case RM_CMD_DART_INFO:             // 0x0105
        memcpy(&dart_info_0x0105, (void *)RMRefereeSystemData.data, sizeof(dart_info_0x0105));
        break;

    /* ---- 机器人状态 ---- */
    case RM_CMD_ROBOT_STATUS:          // 0x0201
        memcpy(&robot_status_0x0201, (void *)RMRefereeSystemData.data, sizeof(robot_status_0x0201));
        break;
    case RM_CMD_POWER_HEAT_DATA:       // 0x0202
        memcpy(&power_heat_data_0x0202, (void *)RMRefereeSystemData.data, sizeof(power_heat_data_0x0202));
        break;
    case RM_CMD_ROBOT_POSITION:        // 0x0203
        memcpy(&robot_pos_0x0203, (void *)RMRefereeSystemData.data, sizeof(robot_pos_0x0203));
        break;
    case RM_CMD_ROBOT_BUFF:            // 0x0204
        memcpy(&buff_0x0204, (void *)RMRefereeSystemData.data, sizeof(buff_0x0204));
        break;
    case RM_CMD_ROBOT_HURT:            // 0x0206
        memcpy(&hurt_data_0x0206, (void *)RMRefereeSystemData.data, sizeof(hurt_data_0x0206));
        break;
    case RM_CMD_SHOOT_DATA:            // 0x0207
        memcpy(&shoot_data_0x0207, (void *)RMRefereeSystemData.data, sizeof(shoot_data_0x0207));
        break;
    case RM_CMD_PROJECTILE_ALLOWANCE:  // 0x0208
        memcpy(&projectile_allowance_0x0208, (void *)RMRefereeSystemData.data, sizeof(projectile_allowance_0x0208));
        break;
    case RM_CMD_RFID_STATUS:           // 0x0209
        memcpy(&rfid_status_0x0209, (void *)RMRefereeSystemData.data, sizeof(rfid_status_0x0209));
        break;
    case RM_CMD_DART_CLIENT_CMD:       // 0x020A
        memcpy(&dart_client_cmd_0x020A, (void *)RMRefereeSystemData.data, sizeof(dart_client_cmd_0x020A));
        break;
    case RM_CMD_GROUND_ROBOT_POSITION: // 0x020B
        memcpy(&ground_robot_position_0x020B, (void *)RMRefereeSystemData.data, sizeof(ground_robot_position_0x020B));
        break;
    case RM_CMD_SENTRY_MARK_INFO:      // 0x020C
        memcpy(&radar_mark_data_0x020C, (void *)RMRefereeSystemData.data, sizeof(radar_mark_data_0x020C));
        break;
    case RM_CMD_SENTRY_INFO:           // 0x020D
        memcpy(&sentry_info_0x020D, (void *)RMRefereeSystemData.data, sizeof(sentry_info_0x020D));
        break;
    case RM_CMD_RADAR_INFO:            // 0x020E
        memcpy(&radar_info_0x020E, (void *)RMRefereeSystemData.data, sizeof(radar_info_0x020E));
        break;

    /* ---- 0x0301~0x0311 机器人交互 / 图传链路 ---- */
    /* 这些命令码的数据长度不固定或需要二次业务处理，按需在此处添加 case 分支 */
    default:
        break;
    }
}


/************************************************************************
 * @brief  裁判系统串口数据接收状态机（按字节喂入）
 * @param  MypData  当前接收到的一个字节
 *
 * 按照协议帧格式：frame_header(5) + cmd_id(2) + data(n) + frame_tail(2)
 * 1. 等待 SOF 字节 0xA5
 * 2. 收够 5 字节帧头后，验证 CRC8
 * 3. 根据 data_length 计算整帧长度
 * 4. 收够整帧后，验证 CRC16
 * 5. CRC16 校验通过，调用 RMRefereeSystemParseData 解析
 ************************************************************************/
static void RMRefereeSystemGetData(uint8_t MypData)
{
    static uint8_t  rx_buf[128] = {0};  // 帧缓存（128 字节足以覆盖常规链路所有命令码）
    static uint16_t rx_index    = 0;    // 当前写入位置
    static uint16_t frame_len   = 0;    // 预期帧总长度

    /* ---- 第 0 字节：等待 SOF ---- */
    if (rx_index == 0)
    {
        if (MypData == 0xA5)
        {
            rx_buf[rx_index++] = MypData;
        }
        return;
    }

    /* ---- 缓存当前字节 ---- */
    rx_buf[rx_index++] = MypData;

    /* ---- 收够帧头（5 字节）：校验 CRC8 并计算帧长 ---- */
    if (rx_index == 5)
    {
        if (!Verify_CRC8_Check_Sum(rx_buf, 5))
        {
            rx_index  = 0;
            frame_len = 0;
            return;  // 帧头 CRC8 校验失败，丢弃并重新寻找 SOF
        }

        uint16_t data_length = (uint16_t)(rx_buf[1] | (rx_buf[2] << 8));
        frame_len            = (uint16_t)(5 + 2 + data_length + 2);  // 帧头 + cmd_id + data + CRC16

        if (frame_len > sizeof(rx_buf))
        {
            rx_index  = 0;
            frame_len = 0;
            return;  // 帧长度超出缓冲区，丢弃
        }
    }

    /* ---- 收够整帧：校验 CRC16 并解析 ---- */
    if (frame_len > 0 && rx_index >= frame_len)
    {
        if (Verify_CRC16_Check_Sum(rx_buf, frame_len))
        {
            RMRefereeSystemParseData(rx_buf, frame_len);
        }

        rx_index  = 0;
        frame_len = 0;
    }
}


/************************************************************************
 * @brief  裁判系统解析入口（在串口中断回调中调用）
 *
 * 使用方式：
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 *   {
 *       if (huart->Instance == HuartDistance_RMRefereeSystem)
 *           RMRefereeSystemParse();
 *   }
 ************************************************************************/
void RMRefereeSystemParse(void)
{
    RMRefereeSystemGetData(MyRefereeSys8Data);
    HAL_UART_Receive_IT(&HuartHandle_RMRefereeSystem, &MyRefereeSys8Data, sizeof(MyRefereeSys8Data));
}
