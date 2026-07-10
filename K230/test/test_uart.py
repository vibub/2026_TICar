from machine import UART, FPIOA
import time

# 改成 K230 开发板上实际连接到 MSPM0 的 IO 编号
K230_UART_TX_IO = 3
K230_UART_RX_IO = 4

fpioa = FPIOA()

# 将物理 IO 映射到 UART1
fpioa.set_function(K230_UART_TX_IO, FPIOA.UART1_TXD)
fpioa.set_function(K230_UART_RX_IO, FPIOA.UART1_RXD)

print("UART1 TX IO:", fpioa.get_pin_num(FPIOA.UART1_TXD))
print("UART1 RX IO:", fpioa.get_pin_num(FPIOA.UART1_RXD))

uart = UART(
    UART.UART1,
    baudrate=115200,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
    timeout=100
)

while True:
    uart.write(b"hello k230\n")
    print("send")

    time.sleep(0.2)

    data = uart.read()
    if data:
        print("recv:", data)

    time.sleep(1)
