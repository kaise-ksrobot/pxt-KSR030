/**
 * KSR030 V0.010
 */
//% weight=10 color=#00A6F0 icon="\uf085" block="KSR030"
namespace KSR030 {
    
    const SERVOMIN = 104 // this is the 'minimum' pulse length count (out of 4096)
    const SERVOMAX = 510 // this is the 'maximum' pulse length count (out of 4096)
    const IIC_ADDRESS = 0x40
    const MODE1 = 0x00
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06

    
    export enum ServoNum {
        S0 = 0,
        S1 = 1,
        S2 = 2,
        S3 = 3,
        S4 = 4,
        S5 = 5,
        S6 = 6,
        S7 = 7,
        S8 = 8,
        S9 = 9,
        S10 = 10,
        S11 = 11,
     }
    export enum MotorNum {
        //% blockId="M1A" block="Right"
        M1A = 0,
        //% blockId="M1B" block="Left"
		M1B = 1,

    }
    export enum LedNum {
        //% blockId="Left_LED" block="Left"
        L_LED = 0,
        //% blockId="Right_LED" block="Right"
		R_LED = 1,

    }
    export enum Track{
        //% blockId="TrackLeft" block="Left"
        Left=0,
        //% blockId="TrackRight" block="Right"
        Right=1,
    }

    export enum RunState {
        //% blockId="Go_Forward" block="Forward"
        Forward = 0,
        //% blockId="Car_Back" block="Back"
        Back = 1,
        //% blockId="Go_Left" block="Left"
        Left = 2,
        //% blockId="GO_Right" block="Right"
        Right = 3,
        //% blockId="Go_Stop" block="Stop"
        Stop = 4,
     
    }
    export enum FrqState {
        //% blockId="Frq_A" block="A"
        A = 0,
        //% blockId="Frq_B" block="B"
        B = 1,
        //% blockId="Frq_C" block="C"
        C = 2,
        //% blockId="Frq_D" block="D"
        D = 3,
        //% blockId="Frq_E" block="E"
        E = 4,
        //% blockId="Frq_F" block="F"
        F = 5,
     
    }




    let initialized = false;
    let neoStrip: neopixel.Strip;
	
    function i2c_write(reg: number, value: number) {
        
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(IIC_ADDRESS, buf)
    }

    function i2c_read(reg: number){
        
        pins.i2cWriteNumber(IIC_ADDRESS, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(IIC_ADDRESS, NumberFormat.UInt8BE);
        return val;
    }

    function init(): void {
        pins.setPull(DigitalPin.P8, PinPullMode.PullUp);
        pins.setPull(DigitalPin.P12, PinPullMode.PullUp);
        i2c_write(MODE1, 0x00);
        // Constrain the frequency
        setFreq(50);
        initialized = true;
    }

    function setFreq(freq: number): void {
        let prescaleval = 25000000/4096/freq;
        prescaleval -= 1;
        let prescale = prescaleval; 
        //let prescale = 121;
        let oldmode = i2c_read(MODE1);        
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2c_write(MODE1, newmode); // go to sleep
        i2c_write(PRESCALE, prescale); // set the prescaler
        i2c_write(MODE1, oldmode);
        control.waitMicros(5000);
        i2c_write(MODE1, oldmode | 0xa0);
	}
	
	function setPwm(channel: number, on: number, off: number): void {
		if (channel < 0 || channel > 15)
            return;

        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on>>8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off>>8) & 0xff;
        pins.i2cWriteBuffer(IIC_ADDRESS, buf);
	}	

	function servo_map(x: number, in_min: number, in_max: number, out_min: number, out_max: number)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    function motor_map(x: number)
    {
        x = x*16; // map 255 to 4096
		if(x > 4095){
			x= 4095;
		}
		if(x < -4095){
			x= -4095;
		}
        return x;
    }

    //% blockId=KSR030_Frq_Set
    //% block="PWM Frequency Set %frqval"
    //% weight=99
    export function Frq_Set(frqval:FrqState):void{
       
        if(!initialized){
			init()
		}
        if(frqval==FrqState.A){
            i2c_write(MODE1, 0x00);
            // Constrain the frequency
            setFreq(50*0.92);

        }else if(frqval==FrqState.B){
            i2c_write(MODE1, 0x00);
            // Constrain the frequency
            setFreq(50*0.94);

        }else if(frqval==FrqState.C){
            i2c_write(MODE1, 0x00);
            // Constrain the frequency
            setFreq(50*0.96);
            
        }else if(frqval==FrqState.D){
            i2c_write(MODE1, 0x00);
            // Constrain the frequency
            setFreq(50*0.98);
            
        }else if(frqval==FrqState.E){
            i2c_write(MODE1, 0x00);
            // Constrain the frequency
            setFreq(50*1);
            
        }else if(frqval==FrqState.F){
            i2c_write(MODE1, 0x00);
            // Constrain the frequency
            setFreq(50*1.02);
            
        }
    }

    //% blockId=KSR030_Ultrasonic 
    //% block="Ultrasonic(cm)"
    //% weight=98
    export function Ultrasonic(): number {

        let maxCmDistance = 500
        // send pulse
        pins.setPull(DigitalPin.P13, PinPullMode.PullNone);
        pins.digitalWritePin(DigitalPin.P13, 0);
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P13, 1);
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P13, 0);

        const d = pins.pulseIn(DigitalPin.P13, PulseValue.High, maxCmDistance * 58);
        // read pulse
        
        return Math.idiv(d, 58);
    }

    
    //% blockId=KSR030_Track
    //% block="Track Sensor %sensor"
    //% weight=97
    export function Read_Track(sensor:Track):number{
        if(!initialized){
			init()
		}
        if(sensor==Track.Left){
            return pins.digitalReadPin(DigitalPin.P12)
        }else if(sensor==Track.Right){
            return pins.digitalReadPin(DigitalPin.P8)
        }else{
            return -1
        } 
    }

    //% blockId="KSR030_RGB" 
    //% block="RGB LED "
    //% weight=96
    export function RGB_LED(): neopixel.Strip {
        if (!neoStrip) {
            neoStrip = neopixel.create(DigitalPin.P16, 2, NeoPixelMode.RGB)
           
        }
 
        return neoStrip;
    }

    /**
     * Used to move the given servo to the specified degrees (0-180) connected to the KSR030
     * @param channel The number (1-16) of the servo to move
     * @param degrees The degrees (0-180) to move the servo to 
     */
    //% blockId=KSR030_Servo
    //% block="Servo channel %channel|degrees %degree"
    //% weight=86
    //% degree.min=0 degree.max=180
	export function Servo(channel: ServoNum, degree: number): void {
        
        if(!initialized){
			init()
		}
		// 50hz: 20,000 us
        //let servo_timing = (degree*1800/180+600) // 0.55 ~ 2.4
        //let pulselen = servo_timing*4096/20000
        //normal 0.5ms~2.4ms
        //SG90 0.5ms~2.0ms

        let pulselen = servo_map(degree, 0, 180, SERVOMIN, SERVOMAX);
        //let pulselen = servo_map(degree, 0, 180, servomin, servomax);
        setPwm(channel, 0, pulselen);
  
    }
    
	/**
     * Used to move the given servo to the specified degrees (0-180) connected to the KSR030
     * @param channel The number (1-16) of the servo to move
     * @param degrees The degrees (0-180) to move the servo to
     * @param servomin 'minimum' pulse length count ; eg: 104
     * @param servomax 'maximum' pulse length count ; eg: 510
     */
    //% blockId=KSR030_ServoRange
    //% block="Servo channel %channel|degrees %degree|servomin %servomin|servomax %servomax"
    //% degree.min=0 degree.max=180
	export function ServoRange(channel: ServoNum, degree: number, servomin: number, servomax: number): void {
        
        if(!initialized){
			init()
		}
		// 50hz: 20,000 us
        //normal 0.5ms~2.4ms
        //SG90 0.5ms~2.0ms
        // servomin Servo_min_timing (ms)*1000*4096/20000 
        // servomax Servo_max_timing (ms)*1000*4096/20000 
        // let pulselen = servo_map(degree, 0, 180, SERVOMIN, SERVOMAX);
        let pulselen = servo_map(degree, 0, 180, servomin, servomax);
        setPwm(channel, 0, pulselen);

    }

    //% blockId=KSR030_Motor 
    //% block="Motor channel %channel|speed %speed"
	//% weight=85
	//% speed.min=-255 speed.max=255
	export function Motor(channel: MotorNum, speed: number): void {
		if(!initialized){
			init()
        }
        
        speed=motor_map(speed);
        
		let pwm1 = (channel*2)+12;
		let pwm2 = (channel*2)+13;
		if(speed>=0){
			setPwm(pwm1, 0, speed)
			setPwm(pwm2, 0, 0)
		}else{
			setPwm(pwm1, 0, 0)
			setPwm(pwm2, 0, -speed)
        }
            
    }

    //% blockId=KSR030_Servo_Car
    //% block="Servo_Car %index|L_speed %lspeed|R_speed %rspeed"
    //% weight=88
    //% lspeed.min=0 lspeed.max=90 rspeed.min=0 rspeed.max=90
    export function Servo_Car(index: RunState, lspeed: number, rspeed: number): void {
        let l_pulselen=0;
        let r_pulselen=0;
        if(!initialized){
			init()
		}
        
        switch (index) {
            case RunState.Forward: 
                l_pulselen = servo_map(90+lspeed, 0, 180, SERVOMIN, SERVOMAX);
                r_pulselen = servo_map(90-rspeed, 0, 180, SERVOMIN, SERVOMAX);
                setPwm(ServoNum.S8, 0, l_pulselen);
                setPwm(ServoNum.S9, 0, r_pulselen);
                break;
            case RunState.Back: 
                l_pulselen = servo_map(90-lspeed, 0, 180, SERVOMIN, SERVOMAX);
                r_pulselen = servo_map(90+rspeed, 0, 180, SERVOMIN, SERVOMAX);
                setPwm(ServoNum.S8, 0, l_pulselen);
                setPwm(ServoNum.S9, 0, r_pulselen);
                break;
            case RunState.Left: 
                l_pulselen = servo_map(90+lspeed/4, 0, 180, SERVOMIN, SERVOMAX);
                r_pulselen = servo_map(90-rspeed, 0, 180, SERVOMIN, SERVOMAX);
                setPwm(ServoNum.S8, 0, l_pulselen);
                setPwm(ServoNum.S9, 0, r_pulselen);
                break;
            case RunState.Right: 
                l_pulselen = servo_map(90+lspeed, 0, 180, SERVOMIN, SERVOMAX);
                r_pulselen = servo_map(90-rspeed/4, 0, 180, SERVOMIN, SERVOMAX);
                setPwm(ServoNum.S8, 0, l_pulselen);
                setPwm(ServoNum.S9, 0, r_pulselen);
                break;
            case RunState.Stop: 
                l_pulselen = servo_map(90, 0, 180, SERVOMIN, SERVOMAX);
                r_pulselen = servo_map(90, 0, 180, SERVOMIN, SERVOMAX);
                setPwm(ServoNum.S8, 0, l_pulselen);
                setPwm(ServoNum.S9, 0, r_pulselen);
                break;
            
        }
    }

    //% blockId=KSR030_Motor_Car
    //% block="Motor_Car %index|L_speed %lspeed|R_speed %rspeed"
    //% weight=87
    //% lspeed.min=0 lspeed.max=255 rspeed.min=0 rspeed.max=255
    export function Motor_Car(index: RunState, lspeed: number, rspeed: number): void {
        switch (index) {
            case RunState.Forward: 
                Motor(MotorNum.M1B,lspeed);
                Motor(MotorNum.M1A,rspeed);
                break;
            case RunState.Back: 
                Motor(MotorNum.M1B,-lspeed);
                Motor(MotorNum.M1A,-rspeed);
                break;
            case RunState.Left: 
                Motor(MotorNum.M1B,lspeed/2);
                Motor(MotorNum.M1A,rspeed);
                break;
            case RunState.Right: 
                Motor(MotorNum.M1B,lspeed);
                Motor(MotorNum.M1A,rspeed/2); 
                break;
            case RunState.Stop: 
                Motor(MotorNum.M1B,0);
                Motor(MotorNum.M1A,0);
                break;
            
        }
    }

    //% blockId=PWM_DETECT_Frequency
    //% block="DETECT Servo %channel Frequency to pin %iopin"
    //% weight=80
    export function DETECT_Frequency(channel: ServoNum, iopin: DigitalPin): number  {
        let frq = 0;
        let frqPinState = 0;
        let prevFrqPinState = 0;
        let oneSecond = 1000;
        let timer = 0;
        let ret_frq = 0;
       
        if(!initialized){
			init()
        }
        setPwm(channel, 0, SERVOMAX);
        for(let i=0; i<2000 ; i++) {
            frqPinState = pins.digitalReadPin(iopin)
            if (frqPinState == 0) {
                prevFrqPinState = 0
            }
            if (frqPinState == 1 && prevFrqPinState == 0) {
                prevFrqPinState = frqPinState
                frq = frq + 1
            }
            control.waitMicros(1000)
            timer = timer + 1
            if (timer > oneSecond) {
                frq = frq-2
                if (frq > 53) {
                    //basic.showString("A")
                    ret_frq= 65
                } else {
                    if (frq > 52) {
                        //basic.showString("B")
                        ret_frq= 66
                    } else {
                        if (frq > 51) {
                            //basic.showString("C")
                            ret_frq= 67
                        } else {
                            if (frq > 50) {
                                //basic.showString("D")
                                ret_frq=  68
                            } else {
                                if (frq > 49) {
                                    //basic.showString("E")
                                    ret_frq=  69
                                } else {
                                    if (frq > 48) {
                                        //basic.showString("F")
                                        ret_frq=  70
                                    } else {
                                        if(frq <= 48) {
                                            //basic.showString("X")
                                            ret_frq=  88

                                        }
                                    }
                                        
                                }
                            }
                        }
                    }
                }

                frq = 0
                timer = 0
            }
        }
        return ret_frq
        

    }


}