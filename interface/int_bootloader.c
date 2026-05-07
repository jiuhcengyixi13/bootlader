
/**
 * @file booylader.c
 * @brief Bootloader串口接收与Flash写入实现
 * @details 实现通过串口接收应用程序并写入Flash的功能
 */

#include "Int_bootloader.h"

// 全局变量定义
uint8_t g_uart_rec_buff[BOOTLOADER_UART_REC_BUFF_LEN] = {0}; // 串口接收缓冲区
uint16_t g_uart_rec_len = 0;                                 // 当前接收数据长度
uint16_t g_uart_rec_full_len = 0;                            // 累计接收数据长度
uint32_t g_uart_rec_offset = 0;                              // Flash写入偏移量
uint8_t g_last_byte_flag = 0;                                // 是否有遗留单字节标记
uint8_t g_last_byte = 0;                                     // 保存遗留的单字节

static void Int_flash_erase(void)
{
    // 判断是否需要擦除Flash页（检测目标地址是否全为0xff）
    uint8_t is_erase = 0;
    uint32_t page_addr = 0;
    for (uint16_t i = 0; i < g_uart_rec_len; i++)
    {
        volatile uint8_t *data = (volatile uint8_t *)(APP_START_ADDRESS + i + g_uart_rec_offset);
        if (*data != 0xff)
        {
            printf("erase:%d,%d,%c", i, g_uart_rec_offset, data);
            is_erase = 1;
            // 计算页起始地址（页对齐）
            page_addr = (APP_START_ADDRESS + i + g_uart_rec_offset) -
                        ((APP_START_ADDRESS + i + g_uart_rec_offset) % FLASH_PAGE_SIZE);
            break;
        }
    }

    // 执行Flash页擦除
    if (is_erase)
    {
        FLASH_EraseInitTypeDef erase_init;
        erase_init.TypeErase = FLASH_TYPEERASE_PAGES; // 页擦除
        erase_init.Banks = FLASH_BANK_1;              // 选择Bank 1
        erase_init.PageAddress = page_addr;           // 擦除起始地址
        erase_init.NbPages = 1;                       // 擦除1页
        uint32_t page_error = 0;
        HAL_FLASHEx_Erase(&erase_init, &page_error);
    }
}
/**
 * @brief 带遗留字节的Flash写入函数
 * @details 将上次遗留的单字节与本次数据拼接后写入Flash
 */
static void Int_flash_write_with_last(void)
{
    for (uint16_t i = 0; i < g_uart_rec_len; i += 2)
    {
        uint16_t data16;
        uint32_t flash_addr = APP_START_ADDRESS + i + g_uart_rec_offset;

        if (i == 0) // 第一个半字：拼接遗留字节与当前第一个字节
        {
            // data16 = g_last_byte | (g_uart_rec_buff[i] << 8);
            data16 = (g_uart_rec_buff[i] << 8) | g_last_byte;
        }
        else // 后续半字：拼接当前字节与前一字节
        {
            data16 = (uint16_t)(g_uart_rec_buff[i - 1] | g_uart_rec_buff[i] << 8);
        }

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr, data16);
    }
}

/**
 * @brief 无遗留字节的Flash写入函数
 * @details 直接将数据以半字为单位写入Flash
 */
static void Int_flash_write_no_last(void)
{
    for (uint16_t i = 0; i < g_uart_rec_len; i += 2)
    {
        uint16_t data16;
        uint32_t flash_addr = APP_START_ADDRESS + i + g_uart_rec_offset;
        if (i + 1 < g_uart_rec_len)
        {
            // data16 = (uint16_t)(g_uart_rec_buff[i] << 8 | g_uart_rec_buff[i + 1]);
            data16 = (uint16_t)(g_uart_rec_buff[i + 1] << 8 | g_uart_rec_buff[i]);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr, data16);
        }
    }
}

static void Int_flash_write_halfword(void)
{
    // 根据数据长度奇偶性选择写入方式
    if ((g_uart_rec_len + g_last_byte_flag) % 2 == 0)
    {
        if (g_last_byte_flag)
        {
            // 有遗留字节，且总长度为偶数
            Int_flash_write_with_last();
            g_uart_rec_offset += g_uart_rec_len + 1;
        }
        else
        {
            // 无遗留字节，且长度为偶数
            Int_flash_write_no_last();
            g_uart_rec_offset += g_uart_rec_len;
        }
        g_last_byte_flag = 0;
    }
    else
    {
        if (g_last_byte_flag)
        {
            // 有遗留字节，且总长度为奇数
            Int_flash_write_with_last();
            g_last_byte = g_uart_rec_buff[g_uart_rec_len - 1];
            g_uart_rec_offset += g_uart_rec_len;
        }
        else
        {
            // 无遗留字节，且长度为奇数
            Int_flash_write_no_last();
            g_last_byte = g_uart_rec_buff[g_uart_rec_len - 1];
            g_uart_rec_offset += g_uart_rec_len - 1;
        }
        g_last_byte_flag = 1;
    }
}

/**
 * @brief Bootloader初始化函数
 * @details 初始化串口中断接收，清除标志位，避免启动前接收数据导致溢出
 */
void Int_bootloader_init(void)
{
    // 清除串口溢出标志和空闲帧标志
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    // 启动串口空闲中断接收
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, g_uart_rec_buff, BOOTLOADER_UART_REC_BUFF_LEN);
}
/**
 * @brief 串口空闲中断回调函数
 * @param huart UART句柄
 * @param Size 接收数据长度
 * @details 接收完成后将数据写入Flash，处理奇偶长度数据的拼接
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        // 保存本次接收的数据长度
        g_uart_rec_len = Size;
        g_uart_rec_full_len += g_uart_rec_len;

        // 解锁Flash以便写入
        HAL_FLASH_Unlock();

        // 擦除Flash页
        Int_flash_erase();

        // 根据数据长度奇偶性选择写入16字节方式
        Int_flash_write_halfword();

        // 上锁Flash
        HAL_FLASH_Lock();
    }

    // 清空缓冲区，准备下次接收
    memset(g_uart_rec_buff, 0, BOOTLOADER_UART_REC_BUFF_LEN);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, g_uart_rec_buff, BOOTLOADER_UART_REC_BUFF_LEN);
}

// #include "Int_bootloader.h"

// // 接收程序的缓冲区
// uint8_t g_uart_rec_buff[BOOTLOADER_UART_REC_BUFF_LEN] = {0};
// uint16_t g_uart_rec_len = 0;
// uint16_t g_uart_rec_full_len = 0;
// // 偏移量
// uint32_t g_uart_rec_offset = 0;
// // 末尾可能出现单独一个字节
// uint8_t g_last_byte_flag = 0; // 用来标记是否为最后一个字节
// uint8_t g_last_byte = 0;      // 用来保存最后一个字节

// void Int_flash_writr_with_last(void)
// {
//     for (uint16_t i = 0; i < g_uart_rec_len; i += 2)
//     {
//         uint16_t data16;
//         uint32_t flash_addr = APP_START_ADDRESS + i + g_uart_rec_offset;
//         if (i == 0) // 第一个字节
//         {
//             // 和上一个字节进行拼接
//             data16 = g_last_byte | (g_uart_rec_buff[i] << 8); // 拼接上一次和下标0
//         }
//         else
//         {
//             // 拼接下标1和2
//             data16 = (uint16_t)(g_uart_rec_buff[i - 1] | g_uart_rec_buff[i] << 8);
//         }
//         HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr, data16);
//     }
// }
// void Int_flash_writr_no_last(void)
// {
//     for (uint16_t i = 0; i < g_uart_rec_len; i += 2)
//     {
//         uint16_t data16;
//         uint32_t flash_addr = APP_START_ADDRESS + i + g_uart_rec_offset;
//         data16 = (uint16_t)(g_uart_rec_buff[i] << 8 | g_uart_rec_buff[i + 1]); // 拼接
//         HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr, data16);
//     }
// }

// // 串口接受->准备接受A程序
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

// // 串口开启中断接收之后
// // 触发的空闲帧时使用的回调函数
// // void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// // {
// //     if (huart->Instance == USART1)
// //     {
// //         // 保存实际接受的数据长度
// //         uart_rec_len = Size;
// //         // printf()底层就是调用的fputc——>重定向为串口输出
// //         // 可以实现rizh
// //         printf("buff:%s", uart_rec_buff);
// //         // TODO:将接受的的数据写入flash

// //         // 使用完数据之后 清空 准备下一次的接收
// //         memset(uart_rec_buff, 0, BOOTLOADER_UART_REC_BUFF_LEN);
// //         HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart_rec_buff, BOOTLOADER_UART_REC_BUFF_LEN);
// //     }
// // }

// // 主函数
// void Int_bootloader_init(void)
// {
//     // 先清空到初始化串口使用之前的问题
//     // 避免硬件再次软件开启前就源源不断收到信息导致溢出
//     __HAL_UART_CLEAR_OREFLAG(&huart1);
//     __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//     // 带有中断得的串口接受函数
//     // 少一个参数 超时时间 因为IT带中断的函数方法是异步执行的
//     // ***初始化之后用串口接收长数据，一旦触发到空闲帧（也就是接收完成就出发下面的函数）

//     HAL_UARTEx_ReceiveToIdle_IT(&huart1, g_uart_rec_buff, BOOTLOADER_UART_REC_BUFF_LEN);
// }
// // 触发空闲帧之后打印数据和写入flash
// // 总长度接受几千个字节 串口协议稳定性差 发送长文件的时候 容易丢失字节
// // 使用 DMA传输能优化一些，修改波特率9600能提高稳定性
// // 系统自带的串口接收回调函数（把中断函数的结果回调）原本是弱函数，写在这里提高优先级
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     if (huart->Instance == USART1)
//     {
//         // 需要延时时间来执行下列的代码
//         //  保存实际接受的数据长度
//         g_uart_rec_len = Size;
//         g_uart_rec_full_len += g_uart_rec_len;
//         // ***printf()底层就是调用的fputc——>重定向为串口输出
//         // ***可以实现printf 用来打印空闲帧，底层是
//         // ***HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//         // printf("buff:%d", uart_rec_full_len);在中断回调函数中用printf会很影响性能和稳定性

//         // TODO:将接受的的数据写入flash
//         // 1.flash默认上锁，得先解锁
//         HAL_FLASH_Unlock();
//         // 2.判断当前写入的地址是否为新的一页 =>是就擦除flash
//         // 判断当前写入的256字节是否都为ff不是就擦除
//         // 遍历与要写入的地址，长度为当前接受的数据长度，如果内存全部为0xff则说明已经擦除
//         uint8_t is_erase = 0;
//         uint32_t page_addr = 0;
//         for (uint16_t i = 0; i < g_uart_rec_len; i++)
//         {
//             // 2.1每次读取一个位置得值
//             // 用data指针指向当前写入的地址 (volatile uint8_t *)强制转化为指针类型好赋值给前面
//             // volatile告诉系统不要更改优化
//             volatile uint8_t *data = (volatile uint8_t *)(APP_START_ADDRESS + i + g_uart_rec_offset);
//             if (*data != 0xff)
//             {
//                 is_erase = 1;
//                 page_addr = (APP_START_ADDRESS + i + g_uart_rec_offset) -
//                             (APP_START_ADDRESS + i + g_uart_rec_offset) %
//                                 FLASH_PAGE_SIZE;
//                 break;
//             }
//             // 2.2如果需要擦除，则进行擦除
//         }
//         if (is_erase)
//         {
//             //             typedef struct
//             // {
//             //   uint32_t TypeErase;

//             //   uint32_t Banks;

//             //   uint32_t PageAddress;

//             //   uint32_t NbPages;

//             // } FLASH_EraseInitTypeDef;
//             FLASH_EraseInitTypeDef erase_init;            // 擦除初始化结构体变量
//             erase_init.TypeErase = FLASH_TYPEERASE_PAGES; // 擦除类型 页擦除
//             erase_init.Banks = FLASH_BANK_1;              // 选择的那一块
//             erase_init.PageAddress = page_addr;           // 擦除的起始页地址 页对齐
//             // 擦除几页
//             erase_init.NbPages = 1;
//             uint32_t page_error = 0;
//             // flash擦除比较耗费性能
//             HAL_FLASHEx_Erase(&erase_init, &page_error);
//         }

//         // 2.3用16位写入数据 另两个字节

//         // 判断当前写入得内容是否为偶数有四种情况
//         if ((g_uart_rec_len + g_last_byte_flag) % 2 == 0)
//         {
//             if (g_last_byte_flag)
//             {
//                 // 1+5，上次遗留了一个字节，遗留的这次要作为第一个字节
//                 Int_flash_writr_with_last();
//                 // for (uint16_t i = 0; i < g_uart_rec_len; i += 2)
//                 // {
//                 //     uint16_t data16;
//                 //     uint32_t flash_addr = APP_START_ADDRESS + i + g_uart_rec_offset;
//                 //     if (i == 0) // 第一个字节
//                 //     {
//                 //         // 和上一个字节进行拼接
//                 //         data16 = g_last_byte | (g_uart_rec_buff[i] << 8); // 拼接上一次和下标0
//                 //     }
//                 //     else
//                 //     {
//                 //         // 拼接下标1和2
//                 //         data16 = (uint16_t)(g_uart_rec_buff[i - 1] | g_uart_rec_buff[i] << 8);
//                 //     }
//                 // }
//                 // 2.4 记录偏移量
//                 g_uart_rec_offset += g_uart_rec_len + 1;
//             }
//             else
//             {
//                 // 0+4,这次是偶数，而且上次没有遗留，直接写入
//                 Int_flash_writr_no_last();
//                 // for (uint16_t i = 0; i < g_uart_rec_len; i += 2)
//                 // {
//                 //     uint16_t data16;
//                 //     uint32_t flash_addr = APP_START_ADDRESS + i + g_uart_rec_offset;
//                 //     data16 = (uint16_t)(g_uart_rec_buff[i] << 8 | g_uart_rec_buff[i + 1]); // 拼接
//                 //     HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr, data16);
//                 // }
//                 // 2.4 记录偏移量
//                 g_uart_rec_offset += g_uart_rec_len;
//             }
//             g_last_byte_flag = 0;
//         }
//         else // 奇数
//         {
//             if (g_last_byte_flag)
//             {
//                 // 1+4,上次遗留了一个字节，这次是偶数
//                 Int_flash_writr_with_last();
//                 // for (uint16_t i = 0; i < g_uart_rec_len; i += 2)
//                 // {
//                 //     uint16_t data16;
//                 //     uint32_t flash_addr = APP_START_ADDRESS + i + g_uart_rec_offset;
//                 //     if (i == 0) // 第一个字节
//                 //     {
//                 //         // 和上一个字节进行拼接
//                 //         data16 = g_last_byte | (g_uart_rec_buff[i] << 8); // 拼接上一次和下标0
//                 //     }
//                 //     else
//                 //     {
//                 //         // 拼接下标1和2
//                 //         data16 = (uint16_t)(g_uart_rec_buff[i - 1] | g_uart_rec_buff[i] << 8);
//                 //     }
//                 // }
//                 // 修改最后一个字节
//                 g_last_byte = g_uart_rec_buff[g_uart_rec_len - 1];
//                 // 偏移量
//                 g_uart_rec_offset += g_uart_rec_len;
//             }
//             else
//             {
//                 // 0+5，这次是奇数，上次没有遗留，这次会留下一个
//                 Int_flash_write_no_last();
//                 // for (uint16_t i = 0; i < g_uart_rec_len; i += 2)
//                 // {
//                 //     uint16_t data16;
//                 //     uint32_t flash_addr = APP_START_ADDRESS + i + g_uart_rec_offset;
//                 //     data16 = (uint16_t)(g_uart_rec_buff[i] << 8 | g_uart_rec_buff[i + 1]); // 拼接
//                 //     HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr, data16);
//                 // }
//                 // 修改最后一个字节
//                 // 有遗留了
//                 g_last_byte = g_uart_rec_buff[g_uart_rec_len - 1]; // 吧最后遗留的字节赋值给last_byte
//                                                                    // 偏移量
//                 g_uart_rec_offset += g_uart_rec_len - 1;
//             }
//             g_last_byte_flag = 1; // 本次之后有剩余
//         }

//         // for (uint16_t i = 0; i < g_uart_rec_len; i += 2)
//         // {
//         //     uint16_t data16;
//         //     uint32_t flash_addr = APP_START_ADDRESS + i + g_uart_rec_offset;
//         //     if (i + 1 < g_uart_rec_len)
//         //     {
//         //         data16 = (uint16_t)(g_uart_rec_buff[i] << 8 | g_uart_rec_buff[i + 1]); // 拼接
//         //     }

//         //     else
//         //     {
//         //         // 最后一个字节是单独的情况
//         //         // 该程序有bug，当hello写入flash后是he，ll，off，我们下次在发送的时候本想让h吧ff覆盖
//         //         // 但是因为flash想直接写入不能实现就会卡死出bug
//         //         data16 = g_uart_rec_buff[i] | (0xff << 8);
//         //     }
//         //     HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr, data16);
//         // }

//         // // 2.4 记录偏移量
//         // g_uart_rec_offset += g_uart_rec_len;
//         // 3.上锁flash
//         HAL_FLASH_Lock();
//     }
//     // 使用完数据之后 清空 准备下一次的接收
//     memset(g_uart_rec_buff, 0, BOOTLOADER_UART_REC_BUFF_LEN);
//     __HAL_UART_CLEAR_OREFLAG(&huart1);
//     __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//     HAL_UARTEx_ReceiveToIdle_IT(&huart1, g_uart_rec_buff, BOOTLOADER_UART_REC_BUFF_LEN);
// }
