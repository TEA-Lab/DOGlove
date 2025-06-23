# Getting Started

```{toctree}
:hidden:
:maxdepth: 1

../quick_start/01_pcba.rst
../quick_start/02_3d_print.rst
../quick_start/03_flash_firmware.rst
../quick_start/05_make_it_move.rst
```

## Before You Start

1. Check our [hardware bill of materials (BOM)](#hardware-bill-of-materials) to make sure you have everything.
2. [Order the PCBA](../quick_start/01_pcba).
3. [Print the mechanical structures](../quick_start/02_3d_print).
4. The whole system has been tested under **Ubuntu 20.04**.

(hardware-bill-of-materials)=
## Hardware Bill of Materials

| Component                                | Quantity | Purchase Link | Purchase Link (China) |
| :--------------------------------------- | -------: | :------------ | :-------------------- |
| **3D Print Filament**                    |          |               |                       |
| Bambu PETG Basic (Gold)                  |        1 |[Bambu Store](https://us.store.bambulab.com/products/petg-hf?id=42735939649672)|[Taobao](https://detail.tmall.com/item.htm?id=816799102364)|
| Bambu PETG HF (White)                    |        1 |[Bambu Store](https://us.store.bambulab.com/products/petg-hf?id=42735939846280)|[Taobao](https://detail.tmall.com/item.htm?id=816799102364)|
| Bambu Support for PLA/PETG               |        1 |[Bambu Store](https://us.store.bambulab.com/products/support-for-pla-petg)|[Taobao](https://detail.tmall.com/item.htm?abbucket=9&id=813808728827)|
| Bambu TPU for AMS (White)                |        1 |[Bambu Store](https://us.store.bambulab.com/products/tpu-for-ams?id=43059884720264)|[Taobao](https://detail.tmall.com/item.htm?abbucket=9&id=847975695749&skuId=5633828620351)|
| **Dynamixel Parts**                      |          |               |                       |
| Dynamixel XC330-M288-T                   |        2 |[Dynamixel Store](https://www.robotis.us/dynamixel-xc330-m288-t/)|<a href="https://item.taobao.com/item.htm?_u=m2pu41081584&id=660146504579&skuId=4941606395637">Taobao</a>|
| Dynamixel XL330-M288-T                   |        3 |[Dynamixel Store](https://www.robotis.us/dynamixel-xl330-m288-t/)|<a href="https://item.taobao.com/item.htm?_u=m2pu410853f0&id=648175925849">Taobao</a>|
| ROBOTIS U2D2 USB Communication Converter |        1 |[Dynamixel Store](https://www.robotis.us/u2d2/)|<a href="https://item.taobao.com/item.htm?_u=m2pu4108eef4&id=564915402895">Taobao</a>|
| ROBOTIS Cable-X3P 100mm                  |        4 |Temporarily unavailable (non-critical)|<a href="https://item.taobao.com/item.htm?_u=m2pu41088bc7&id=559824480883&skuId=3659974074027">Taobao</a>|
| **Mechanical Parts**                     |          |               |                       |
| M2x10mm Button Head Screw                |       12 |[97763A409](https://www.mcmaster.com/97763A409/)|[M2x10十字圆头](https://detail.tmall.com/item.htm?abbucket=9&id=637524754721&skuId=4742801506990)|
| M2x12mm Button Head Screw                |       10 |[92095A455](https://www.mcmaster.com/92095A455/)|[M2x12十字圆头](https://detail.tmall.com/item.htm?abbucket=9&id=637524754721&skuId=4742801506984)|
| M2x20+3mm Single Through Copper Standoff |        1 |[Amazon](https://www.amazon.com/ZZHXSM-320PCS-Hexagonal-Motherboard-Computer/dp/B0DHKTWMPH/)|[M2x20+3单通滚花铜柱](https://detail.tmall.com/item.htm?abbucket=9&id=534855987272&skuId=5775647192461)|
| M2x20mm Dual Through Copper Standoff     |        1 |[As provided above](https://www.amazon.com/ZZHXSM-320PCS-Hexagonal-Motherboard-Computer/dp/B0DHKTWMPH/)|[M2x20双通滚花铜柱](https://detail.tmall.com/item.htm?abbucket=9&id=534855987272&skuId=5942557358119)|
| M2x5mm Self-Tapping Screw                |       20 |[90380A325](https://www.mcmaster.com/90380A325/)|[M2x5十字圆头割尾自攻](https://detail.tmall.com/item.htm?abbucket=9&id=535345606154&skuId=4327300926628)|
| M2x6mm Self-Tapping Screw                |       16 |(From servo box)|(From servo box)|
| M2x8mm Self-Tapping Screw                |       20 |(From servo box)|(From servo box)|
| M2x5x0.3mm Washer                        |       12 |[Amazon](https://www.amazon.com/Washers-Stainless-Assortment-Sizes%EF%BC%88M2-Industrial/dp/B0CBJZNDN4/)|[M2x5x0.3平垫片](https://detail.tmall.com/item.htm?abbucket=9&id=736651447364&skuId=5800439948793)|
| M2 Lock Nut                              |       10 |[Amazon](https://www.amazon.com/M2-0-3-Insert-Locknuts-Stainless-Clinching/dp/B0DR93VFYG/)|[M2防松螺母](https://detail.tmall.com/item.htm?abbucket=9&id=763443678130&skuId=5254837848373)|
| M2 Hex Nut                               |       10 |[90592A075](https://www.mcmaster.com/90592A075/)|[M2六角螺母](https://detail.tmall.com/item.htm?abbucket=9&id=549640610734&skuId=3507734575864)|
| M3x10mm Button Head Screw                |        2 |[92095A182](https://www.mcmaster.com/92095A182/)|[M3x10十字圆头](https://detail.tmall.com/item.htm?abbucket=9&id=637524754721&skuId=4742801506977)|
| M3x30mm Button Head Screw                |        2 |[92095A187](https://www.mcmaster.com/92095A187/)|[M3x30十字圆头](https://detail.tmall.com/item.htm?abbucket=9&id=637524754721&skuId=4742801506999)|
| M3x15mm D-Shape Shaft Screw              |       16 | —             |<a href="https://item.taobao.com/item.htm?ft=t&id=935960628369">Purchase directly from factory</a>|
| M3x20mm Dual Through Copper Standoff     |        2 |[Amazon](https://www.amazon.com/Csdtylh-Male-Female-Standoff-Stainless-Assortment/dp/B06Y5TJXY1/)|[M3x20双通铜柱](https://detail.tmall.com/item.htm?abbucket=9&id=542213588710&skuId=3258324297349)|
| M3 Lock Nut                              |       12 |[Amazon](https://www.amazon.com/Yinpecly-Stainless-Self-Locking-Industrial-Construction/dp/B0F1MSSVQP/)|[M3防松螺母](https://detail.tmall.com/item.htm?abbucket=9&id=763443678130&skuId=5740530721046)|
| M4x10mm Button Head Screw                |       14 |[92095A190](https://www.mcmaster.com/92095A190/)|[M4x10十字圆头](https://detail.tmall.com/item.htm?abbucket=9&id=637524754721&skuId=4742801506971)|
| M4x15mm Dual Through Copper Standoff     |        4 |[Amazon](https://www.amazon.com/Standoff-Motherboard-Hexagon-Spacer-Assortment/dp/B09X358Q73/)|[M4x15双通铜柱](https://detail.tmall.com/item.htm?abbucket=9&id=542213588710&skuId=4348642093877)|
| M4x20mm Dual Through Copper Standoff     |        1 |[As provided above](https://www.amazon.com/Standoff-Motherboard-Hexagon-Spacer-Assortment/dp/B09X358Q73/)|[M4x20双通铜柱](https://detail.tmall.com/item.htm?abbucket=9&id=542213588710&skuId=4348642093875)|
| M4 Thin-Profile Hex Nut                  |        4 |[90710A035](https://www.mcmaster.com/90710A035/)|[M4扁薄六角螺母](https://detail.tmall.com/item.htm?_u=m2pu410880c4&id=533915610076&skuId=4937683182274)|
| 7x4x2.5mm Bearing                        |       16 |[Amazon](https://www.amazon.com/uxcell-Bearings-4x7x2-5mm-Miniature-Precision/dp/B0CM6RYCC7/)|[MR74-ZZ P5【4x7x2.5】轴承](https://detail.tmall.com/item.htm?_u=m2pu41088525&id=743100333785&skuId=5123565292062)|
| 0.6mm 7x7 Stainless Steel Wire Rope      | 25ft / 10m |[3461T07](https://www.mcmaster.com/3461T07/)|<a href="https://item.taobao.com/item.htm?_u=m2pu4108c68a&id=574480300292&skuId=3929526017813">钢丝绳</a>|
| M3x25mm Headless Clevis Pin              |        5 |[93890A703](https://www.mcmaster.com/93890A703/)|[M3x25双头卡簧销轴](https://detail.tmall.com/item.htm?_u=m2pu41084dab&id=755183277159&skuId=5205582448931)|
| M2 Circlip                               |       10 |[Amazon](https://www.amazon.com/Circlip-External-Retaining-Stainless-Assortment/dp/B0BNB39FKY/)|[M2-e型卡簧挡圈](https://detail.tmall.com/item.htm?_u=m2pu41083fee&id=557748539457&skuId=5002991824253)|
| M4x25mm Headless Clevis Pin              |        5 |[93890A707](https://www.mcmaster.com/93890A707/)|[M4x25双头卡簧销轴](https://detail.tmall.com/item.htm?_u=m2pu41084dab&id=755183277159&skuId=5205582448938)|
| M3 Circlip                               |       10 |[As provided above](https://www.amazon.com/Circlip-External-Retaining-Stainless-Assortment/dp/B0BNB39FKY/)|[M3-e型卡簧挡圈](https://detail.tmall.com/item.htm?_u=m2pu41083fee&id=557748539457&skuId=4902892140873)|
| **Electronics**                          |          |               |                       |
| RDC506 Encoder                           |       16 |[DigiKey](https://www.digikey.com/en/products/detail/alps-alpine/RDC506018A/19529120), [Mouser](https://www.mouser.com/ProductDetail/Alps-Alpine/RDC506018A?qs=j%252B1pi9TdxUZl0Obrq5eEAQ%3D%3D)|[LCSC-RDC506002A](https://item.szlcsc.com/116589.html), [LCSC-RDC506018A](https://item.szlcsc.com/364160.html)|
| LRA0825 Haptic Motor                     |        5 |[JYLRA0825Z](https://www.digikey.com/en/products/detail/jie-yi-electronics-limited/JYLRA0825Z/22519426), [LRA0825BC-0167F](https://www.digikey.com/en/products/detail/ineed-motor/LRA0825BC-0167F/21840080)|[LCSC-LD0825BC-0169F](https://item.szlcsc.com/6552539.html)|
| USB to TTL Module                        |        1 |[Amazon](https://www.amazon.com/JESSINIE-Module-CH343-Communication-Microcontroller/dp/B0D6352BS9/)|<a href="https://item.taobao.com/item.htm?_u=m2pu41088297&id=674671646143&skuId=4849832708260">nanoUART</a>|
| **Cables**                               |          |               |                       |
| PH2.0 Pre-Crimped Wire, 15cm (R/Y/B)     |        6 |[Amazon](https://www.amazon.com/Yoeruyo-PH2-0mm-Connector-Pre-Crimped-3Pin-100CM/dp/B0C2HHHNX9/)|<a href="https://item.taobao.com/item.htm?_u=m2pu4108e868&id=596387029707&skuId=4317745623497">PH2.0镀金单头端子线，电子线15cm，红/黄/黑</a>|
| PH2.0 Pre-Crimped Wire, 25cm (R/Y/B)     |        5 |[Amazon](https://www.amazon.com/Yoeruyo-PH2-0mm-Connector-Pre-Crimped-3Pin-100CM/dp/B0C2HBL668/)|<a href="https://item.taobao.com/item.htm?_u=m2pu4108e868&id=596387029707&skuId=4317745623533">PH2.0镀金单头端子线，电子线25cm，红/黄/黑</a>|
| PH2.0 Pre-Crimped Wire, 30cm (R/Y/B)     |        5 |[Amazon](https://www.amazon.com/Yoeruyo-PH2-0mm-Connector-Pre-Crimped-3Pin-100CM/dp/B0C2HBL668/)|<a href="https://item.taobao.com/item.htm?_u=m2pu4108e868&id=596387029707&skuId=4317745623533">PH2.0镀金单头端子线，电子线30cm，红/黄/黑</a>|
| PH2.0-3P Housing                         |       16 |As provided above|<a href="https://item.taobao.com/item.htm?id=15486830256&spm=a1z09.2.0.0.6b202e8dOSJ2GK&skuId=3283156525609">PH2.0-3P胶壳</a>|
| SH1.0 Pre-Crimped Wire, 40cm (R/B)       |        5 |[Amazon](https://www.amazon.com/Cermant-SH1-0mm-Pin-Connector-Female/dp/B0DM238Y63/)|<a href="https://item.taobao.com/item.htm?_u=m2pu41080c07&id=595880379976&skuId=4302450046792">SH1.0镀金单头端子线，特软硅胶线40cm，红/黑</a>|
| SH1.0-2P Housing                         |        5 |As provided above|<a href="https://item.taobao.com/item.htm?_u=m2pu41087701&id=595859715531&skuId=4132094928761">SH1.0-2P胶壳</a>|
| USB-A to MicroUSB Cable, 3m              |        1 |[Amazon](https://www.amazon.com/MaGeek-Premium-Samsung-Motorola-Google/dp/B00WEVG57K/)|[JD](https://item.jd.com/100039185783.html)|
| USB-A to Type-C Cable, 3m                |        1 |[Amazon](https://www.amazon.com/Belkin-BoostCharge-Samsung-Nintendo-Carplay/dp/B08558DD7G/)|[JD](https://item.jd.com/100008391172.html)|
| **Programming Tools**                    |          |               |                       |
| ST-Link V2 Downloader                    |        1 |[Amazon](https://www.amazon.com/AITRIP-Emulator-Downloader-Programmer-STM32F103C8T6/dp/B0D22S8WVX/)|[ST-Link V2下载器](https://detail.tmall.com/item.htm?abbucket=9&id=558866168716&skuId=5146051183964)|
| 2.54-4p Pin Header                       |        1 |[Amazon](https://www.amazon.com/Jabinco-Breakable-Header-Connector-Arduino/dp/B0817JG3XN/)|[2.54-4p单排针](https://detail.tmall.com/item.htm?abbucket=9&id=546772885976&skuId=3398671888945)|
| Dupont Cable F/F                         |        4 |[Amazon](https://www.amazon.com/California-JOS-Breadboard-Optional-Multicolored/dp/B0BRTJXND9/)|[杜邦线母对母](https://detail.tmall.com/item.htm?abbucket=9&id=41254478179&skuId=5755030488371)|
| **Power Supply**                         |          |               |                       |
| MEAN Well RSP-320-5V Power Supply        |        1 |[Amazon](https://www.amazon.com/Mean-RSP-320-5-Power-Supply-Signs/dp/B00IWC2RLS/)|[明纬RSP-320-5V](https://detail.tmall.com/item.htm?_u=i2pu41088bc7&id=547752759011&skuId=4572439414905)|
| Emergency Stop Button                    |        1 |[Amazon](https://www.amazon.com/Emergency-Button-Momentary-PushButton-Switches/dp/B0CWV7L81B/)|[施耐德XALJ01C急停开关](https://item.jd.com/100114884831.html)|
| Power Cord (Type I, 2m)                  |        1 |[Amazon](https://www.amazon.com/StarTech-com-NEMA-Power-Cord-Volts/dp/B002VY53QE/)|[品字电源线](https://item.jd.com/100132121492.html)|
| **Accessories**                          |          |               |                       |
| UV Resin Glue                            |        1 |[Amazon](https://www.amazon.com/dp/B0BJ65T85R/)|[Ergo 8500](https://item.jd.com/100045475837.html)|
| Super Glue                               |        1 |[Amazon](https://www.amazon.com/Scotch-Super-Liquid-Ounces-AD114/dp/B001PILFVY/)|[Ergo 5910](https://item.jd.com/100198857334.html)|
| Zip Tie 2.5mmx15cm                       |       50 |[Amazon](https://www.amazon.com/KOOWIN-Small-Nylon-Plastic-Cable/dp/B08HMPBY3L/)|[扎带2.5x200mm](https://item.jd.com/100023555528.html)|
| Cable Ties Reusable 9mmx15cm             |        6 |[Amazon](https://www.amazon.com/CableCreation-Fastening-Organizer-Adjustable-Management/dp/B07CWLHRQP/)|[魔术贴9mm*15cm](https://item.jd.com/100002860615.html)|
| Hook and Loop Reusable Fastening Cable Tie 20mmx20cm |        1 |[Amazon](https://www.amazon.com/Inches-Reusable-Fastening-Straps-Wisdompro/dp/B01M1L1YHO/)|[带扣魔术贴20mm*25cm](https://item.jd.com/100120892220.html)|
