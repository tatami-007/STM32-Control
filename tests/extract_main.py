"""直接提取main.c的用户代码区做HAL模型测试，避免维护第二份控制实现。"""
import pathlib,re
source=pathlib.Path('Core/Src/main.c').read_text(encoding='utf-8')
out=pathlib.Path('build/host-tests');out.mkdir(parents=True,exist_ok=True)
def block(name):
    return re.search(r'/\* USER CODE BEGIN '+re.escape(name)+r' \*/(.*?)/\* USER CODE END '+re.escape(name)+r' \*/',source,re.S)[1]
init=block('2').replace('&htim2','pwm').replace('&htim3','enc').replace('&huart1','uart')
init=init.replace('ins_data_t *ins =','ins =')
loop=block('3').strip()
assert loop.endswith('}')
loop=loop[:-1].replace('HAL_Delay(1);','').replace('&htim3','test_encoder') # 模型自身推进1ms
text='#include "main.h"\n'+block('Includes')+block('PV')+block('PFP')+block('0')+block('4')
text+='\nstatic ins_data_t *ins;\nstatic TIM_HandleTypeDef *test_encoder;\nvoid Test_Init(TIM_HandleTypeDef *pwm,TIM_HandleTypeDef *enc,UART_HandleTypeDef *uart){test_encoder=enc;'+init+'}\n'
text+='void Test_Loop(void){'+loop+'}\n'
(out/'main_host.c').write_text(text,encoding='utf-8')
(out/'test_main.h').write_text('#include "stm32f1xx_hal.h"\nvoid Test_Init(TIM_HandleTypeDef*,TIM_HandleTypeDef*,UART_HandleTypeDef*);\nvoid Test_Loop(void);\n',encoding='utf-8')
