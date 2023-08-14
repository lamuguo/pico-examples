import machine
import utime
import rp2

# 定义PIO汇编代码
@rp2.asm_pio(
    autopull=True,      # 自动从FIFO中拉取数据
    pull_thresh=8       # 每8位进行一次拉取
)
def sample_signal():
    in_(rp2.PINS, 1)
    push()

# 初始化状态机
sm = rp2.StateMachine(0, sample_signal, freq=500000, in_base=machine.Pin(19))

# 创建一个简单的中断处理函数，该函数将在数据被推入RX FIFO时触发
def irq_handler(pin):
    data = sm.get()  # 从RX FIFO获取数据
    print("Received data:", data)

# 设置中断
irq_pin = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_DOWN)
irq_pin.irq(trigger=machine.Pin.IRQ_FALLING, handler=irq_handler)

# 启动状态机
sm.active(1)

try:
    while True:
        utime.sleep(1)
except KeyboardInterrupt:
    sm.active(0)  # 如果按下Ctrl+C，停止状态机
