enum pins{
//%
}
}
//% color="#AA278D" iconWidth=50 iconHeight=40
namespace keypad4x4  {
    //% block="读取红外遥控码: [NUM]" blockType="reporter"
    //% NUM.shadow="dropdown" NUM.options="PIN_DigitalRead"
    export function KeypadRead(parameter: any, block: any) {
        let pin = parameter.NUM.code
        Generator.addInclude("YM_IRremote_init", "#include <YoungMakerPort.h> \n #include <YoungMakerIR.h>");
        Generator.addObject(`YM_IR${pin}`, `YoungMakerIR`, `ir_YM${pin}(${pin})`);
        Generator.addCode(`ir_YM${pin}.getCode()`)
    }


}
