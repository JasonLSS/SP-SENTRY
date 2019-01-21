# DMA Channels

### DMA1
|Channel | Stream 0         | Stream 1           | Stream 2         | Stream 3           | Stream 4           | Stream 5    | Stream 6          | Stream 7          |
|  --    | --               | --                 | --               | --                 | --                 | --          | --                | --                |
| 0      | SPI3_RX          |                    | SPI3_RX          | SPI2_RX            | SPI2_TX            | SPI3_TX     |                   | SPI3_TX           |
| 1      | I2C1_RX          |                    | TIM7_UP          |                    | TIM7_UP            | I2C1_RX     | I2C1_TX           | I2C1_TX           |
| 2      | TIM4_CH1         |                    | I2S3_EXT_RX      | TIM4_CH2           | I2S2_EXT_TX        | I2S3_EXT_TX | TIM4_UP           | TIM4_CH3          |
| 3      | I2S3_EXT_RX      | TIM2_UP TIM2_CH3   | I2C3_RX          | I2S2_EXT_RX        | I2C3_TX            | TIM2_CH1    | TIM2_CH2 TIM2_CH4 | TIM2_UP TIM2_CH4  |
| 4      | UART5_RX         | USART3_RX          | UART4_RX         | USART3_TX          | UART4_TX           | USART2_RX   | USART2_TX         | UART5_TX          |
| 5      | UART8_TX         | UART7_TX           | TIM3_CH4 TIM3_UP | UART7_RX           | TIM3_CH1 TIM3_TRIG | TIM3_CH2    | UART8_RX          | TIM3_CH3          |
| 6      | TIM5_CH3 TIM5_UP | TIM5_CH4 TIM5_TRIG | TIM5_CH1         | TIM5_CH4 TIM5_TRIG | TIM5_CH2           |             | TIM5_UP           |                   |
| 7      |                  | TIM6_UP            | I2C2_RX          | I2C2_RX            | USART3_TX          | DAC1        | DAC2              | I2C2_TX           |



### DMA2
|Channel | Stream 0  | Stream 1  | Stream 2                   | Stream 3  | Stream 4                    | Stream 5  | Stream 6                   | Stream 7                    |
| --     | --        | --        | --                         | --        | --                          | --        | --                         | --                          |
| 0      | ADC1      | SAI1_A    | TIM8_CH1 TIM8_CH2 TIM8_CH3 | SAI1_A    | ADC1                        | SAI1_B    | TIM1_CH1 TIM1_CH2 TIM1_CH3 |                             |
| 1      |           | DCMI      | ADC2                       | ADC2      | SAI1_B                      | SPI6_TX   | SPI6_RX                    | DCMI                        |
| 2      | ADC3      | ADC3      |                            | SPI5_RX   | SPI5_TX                     | CRYP_OUT  | CRYP_IN                    | HASH_IN                     |
| 3      | SPI1_RX   |           | SPI1_RX                    | SPI1_TX   |                             | SPI1_TX   |                            |                             |
| 4      | SPI4_RX   | SPI4_TX   | USART1_RX                  | SDIO      |                             | USART1_RX | SDIO                       | USART1_TX                   |
| 5      |           | USART6_RX | USART6_RX                  | SPI4_RX   | SPI4_TX                     |           | USART6_TX                  | USART6_TX                   |
| 6      | TIM1_TRIG | TIM1_CH1  | TIM1_CH2                   | TIM1_CH1  | TIM1_CH4 TIM1_TRIG TIM1_COM | TIM1_UP   | TIM1_CH3                   |                             |
| 7      |           | TIM8_UP   | TIM8_CH1                   | TIM8_CH2  | TIM8_CH3                    | SPI5_RX   | SPI5_TX                    | TIM8_CH4 TIM8_TRIG TIM8_COM |