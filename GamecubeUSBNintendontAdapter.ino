/**
 * Nintendont Gamecube to USB Adapter
 * by Pat Hartl
 * Based on code from:
 * Gamecube controller to Nintendo 64 adapter
 * by Andrew Brown
 * MegaJoy
 */

/**
 * To use, hook up the following to the Arduino Duemilanove:
 * Digital I/O 2: Gamecube controller serial line
 * Digital I/O 8: N64 serial line
 * All appropriate grounding and power lines
 * A 1K resistor to bridge digital I/O 2 and the 3.3V supply
 *
 * The pin-out for the N64 and Gamecube wires can be found here:
 * http://svn.navi.cx/misc/trunk/wasabi/devices/cube64/hardware/cube64-basic.pdf
 * Note: that diagram is not for this project, but for a similar project which
 * uses a PIC microcontroller. However, the diagram does describe the pinouts
 * of the gamecube and N64 wires.
 *
 * Also note: the N64 supplies a 3.3 volt line, but I don't plug that into
 * anything.  The arduino can't run off of that many volts, it needs more, so
 * it's powered externally. Additionally, the arduino has its own 3.3 volt
 * supply that I use to power the Gamecube controller. Therefore, only two lines
 * from the N64 are used.
 */

/*
 Copyright (c) 2009 Andrew Brown

 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without
 restriction, including without limitation the rights to use,
 copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following
 conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
 */

#include "pins_arduino.h"
#include "MegaJoy.h"

#define GC_PIN 2
#define GC_PIN_DIR DDRD
// these two macros set arduino pin 2 to input or output, which with an
// external 1K pull-up resistor to the 3.3V rail, is like pulling it high or
// low.  These operations translate to 1 op code, which takes 2 cycles
#define GC_HIGH DDRD &= ~0x04
#define GC_LOW DDRD |= 0x04
#define GC_QUERY (PIND & 0x04)

// 8 bytes of data that we get from the controller. This is a global
// variable (not a struct definition)
static struct {
    // bits: 0, 0, 0, start, y, x, b, a
    unsigned char data1;
    // bits: 1, L, R, Z, Dup, Ddown, Dright, Dleft
    unsigned char data2;
    unsigned char stick_x;
    unsigned char stick_y;
    unsigned char cstick_x;
    unsigned char cstick_y;
    unsigned char left;
    unsigned char right;
} gc_status;

// Zero points for the GC controller stick
static unsigned char zero_x;
static unsigned char zero_y;

static void gc_send(unsigned char *buffer, char length);
static int gc_get();
static void init_gc_controller();
// static void print_gc_status();

#include "crc_table.h"

void setup()
{
  // Serial.begin(115200);

  // Serial.println();
  // Serial.println("Code has started!");
  // Serial.flush();

  setupMegaJoy();

  // Communication with gamecube controller on this pin
  // Don't remove these lines, we don't want to push +5V to the controller
  digitalWrite(GC_PIN, LOW);  
  pinMode(GC_PIN, INPUT);

  init_gc_controller();

  do {
      // Query for the gamecube controller's status. We do this
      // to get the 0 point for the control stick.
      unsigned char command[] = {0x40, 0x03, 0x00};
      gc_send(command, 3);
      // read in data and dump it to gc_raw_dump
      gc_get();
      interrupts();
      zero_x = gc_status.stick_x;
      zero_y = gc_status.stick_y;
      // Serial.print("GC zero point read: ");
      // Serial.print(zero_x, DEC);
      // Serial.print(", ");
      // Serial.println(zero_y, DEC);
      // Serial.flush();
      
      // some crappy/broken controllers seem to give bad readings
      // occasionally. This is a cheap hack to keep reading the
      // controller until we get a reading that is less erroneous.
  } while (zero_x == 0 || zero_y == 0);
  
}

static void init_gc_controller()
{
  // Initialize the gamecube controller by sending it a null byte.
  // This is unnecessary for a standard controller, but is required for the
  // Wavebird.
  unsigned char initialize = 0x00;
  noInterrupts();
  gc_send(&initialize, 1);

  // Stupid routine to wait for the gamecube controller to stop
  // sending its response. We don't care what it is, but we
  // can't start asking for status if it's still responding
  int x;
  for (x=0; x<64; x++) {
      // make sure the line is idle for 64 iterations, should
      // be plenty.
      if (!GC_QUERY)
          x = 0;
  }
}

/**
 * This sends the given byte sequence to the controller
 * length must be at least 1
 * Oh, it destroys the buffer passed in as it writes it
 */
static void gc_send(unsigned char *buffer, char length)
{
    // Send these bytes
    char bits;
    
    // This routine is very carefully timed by examining the assembly output.
    // Do not change any statements, it could throw the timings off
    //
    // We get 16 cycles per microsecond, which should be plenty, but we need to
    // be conservative. Most assembly ops take 1 cycle, but a few take 2
    //
    // I use manually constructed for-loops out of gotos so I have more control
    // over the outputted assembly. I can insert nops where it was impossible
    // with a for loop
    
    asm volatile (";Starting outer for loop");
outer_loop:
    {
        asm volatile (";Starting inner for loop");
        bits=8;
inner_loop:
        {
            // Starting a bit, set the line low
            asm volatile (";Setting line to low");
            GC_LOW; // 1 op, 2 cycles

            asm volatile (";branching");
            if (*buffer >> 7) {
                asm volatile (";Bit is a 1");
                // 1 bit
                // remain low for 1us, then go high for 3us
                // nop block 1
                asm volatile ("nop\nnop\nnop\nnop\nnop\n");
                
                asm volatile (";Setting line to high");
                GC_HIGH;

                // nop block 2
                // we'll wait only 2us to sync up with both conditions
                // at the bottom of the if statement
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              );

            } else {
                asm volatile (";Bit is a 0");
                // 0 bit
                // remain low for 3us, then go high for 1us
                // nop block 3
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\n");

                asm volatile (";Setting line to high");
                GC_HIGH;

                // wait for 1us
                asm volatile ("; end of conditional branch, need to wait 1us more before next bit");
                
            }
            // end of the if, the line is high and needs to remain
            // high for exactly 16 more cycles, regardless of the previous
            // branch path

            asm volatile (";finishing inner loop body");
            --bits;
            if (bits != 0) {
                // nop block 4
                // this block is why a for loop was impossible
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\n");
                // rotate bits
                asm volatile (";rotating out bits");
                *buffer <<= 1;

                goto inner_loop;
            } // fall out of inner loop
        }
        asm volatile (";continuing outer loop");
        // In this case: the inner loop exits and the outer loop iterates,
        // there are /exactly/ 16 cycles taken up by the necessary operations.
        // So no nops are needed here (that was lucky!)
        --length;
        if (length != 0) {
            ++buffer;
            goto outer_loop;
        } // fall out of outer loop
    }

    // send a single stop (1) bit
    // nop block 5
    asm volatile ("nop\nnop\nnop\nnop\n");
    GC_LOW;
    // wait 1 us, 16 cycles, then raise the line 
    // 16-2=14
    // nop block 6
    asm volatile ("nop\nnop\nnop\nnop\nnop\n"
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\n");
    GC_HIGH;

}
/**
 * Complete copy and paste of gc_send, but with the N64
 * pin being manipulated instead.
 * I copy and pasted because I didn't want to risk the assembly
 * output being altered by passing some kind of parameter in
 * (read: I'm lazy... it probably would have worked)
 */

static int gc_get()
{
    // listen for the expected 8 bytes of data back from the controller and
    // and pack it into the gc_status struct.
    asm volatile (";Starting to listen");
    noInterrupts();

    // treat the 8 byte struct gc_status as a raw char array.
    unsigned char *bitbin = (unsigned char*) &gc_status;

    unsigned char retval;

    asm volatile (
            "; START OF MANUAL ASSEMBLY BLOCK\n"
            // r25 is our bit counter. We read 64 bits and increment the byte
            // pointer every 8 bits
            "ldi r25,lo8(0)\n"
            // read in the first byte of the gc_status struct
            "ld r23,Z\n"
            // default exit value is 1 (success)
            "ldi %[retval],lo8(1)\n"

            // Top of the main read loop label
            "L%=_read_loop:\n"

            // This first spinloop waits for the line to go low. It loops 64
            // times before it gives up and returns
            "ldi r24,lo8(64)\n" // r24 is the timeout counter
            "L%=_1:\n"
            "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
            "rjmp L%=_2\n" // line is low. jump to below
            // the following happens if the line is still high
            "subi r24,lo8(1)\n"
            "brne L%=_1\n" // loop if the counter isn't 0
            // timeout? set output to 0 indicating failure and jump to
            // the end
            "ldi %[retval],lo8(0)\n"
            "rjmp L%=_exit\n"
            "L%=_2:\n"

            // Next block. The line has just gone low. Wait approx 2µs
            // each cycle is 1/16 µs on a 16Mhz processor
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"

            // This block left shifts the current gc_status byte in r23,
            // and adds the current line state as the LSB
            "lsl r23\n" // left shift
            "sbic 0x9,2\n" // read PIND2
            "sbr r23,lo8(1)\n" // set bit 1 in r23 if PIND2 is high
            "st Z,r23\n" // save r23 back to memory. We technically only have
            // to do this every 8 bits but this simplifies the branches below

            // This block increments the bitcount(r25). If bitcount is 64, exit
            // with success. If bitcount is a multiple of 8, then increment Z
            // and load the next byte.
            "subi r25,lo8(-1)\n" // increment bitcount
            "cpi r25,lo8(64)\n" // == 64?
            "breq L%=_exit\n" // jump to exit
            "mov r24,r25\n" // copy bitcounter(r25) to r24 for tmp
            "andi r24,lo8(7)\n" // get lower 3 bits
            "brne L%=_3\n" // branch if not 0 (is not divisble by 8)
            "adiw r30,1\n" // if divisible by 8, increment pointer
            "ld r23,Z\n" // ...and load the new byte into r23
            "L%=_3:\n"

            // This next block waits for the line to go high again. again, it
            // sets a timeout counter of 64 iterations
            "ldi r24,lo8(64)\n" // r24 is the timeout counter
            "L%=_4:\n"
            "sbic 0x9,2\n" // checks PIND2
            "rjmp L%=_read_loop\n" // line is high. ready for next loop
            // the following happens if the line is still low
            "subi r24,lo8(1)\n"
            "brne L%=_4\n" // loop if the counter isn't 0
            // timeout? set output to 0 indicating failure and fall through to
            // the end
            "ldi %[retval],lo8(0)\n"


            "L%=_exit:\n"
            ";END OF MANUAL ASSEMBLY BLOCK\n"
            // ----------
            // outputs:
            : [retval] "=r" (retval),
            // About the bitbin pointer: The "z" constraint tells the
            // compiler to put the pointer in the Z register pair (r31:r30)
            // The + tells the compiler that we are both reading and writing
            // this pointer. This is important because otherwise it will
            // allocate the same register for retval (r30).
            "+z" (bitbin)
            // clobbers (registers we use in the assembly for the compiler to
            // avoid):
            :: "r25", "r24", "r23"
            );

    interrupts();
    return retval;
}

static void print_gc_status()
{
    Serial.println();
    Serial.print("Start: ");
    Serial.println(gc_status.data1 & 0x10 ? 1:0);

    Serial.print("Y:     ");
    Serial.println(gc_status.data1 & 0x08 ? 1:0);

    Serial.print("X:     ");
    Serial.println(gc_status.data1 & 0x04 ? 1:0);

    Serial.print("B:     ");
    Serial.println(gc_status.data1 & 0x02 ? 1:0);

    Serial.print("A:     ");
    Serial.println(gc_status.data1 & 0x01 ? 1:0);

    Serial.print("L:     ");
    Serial.println(gc_status.data2 & 0x40 ? 1:0);
    Serial.print("R:     ");
    Serial.println(gc_status.data2 & 0x20 ? 1:0);
    Serial.print("Z:     ");
    Serial.println(gc_status.data2 & 0x10 ? 1:0);

    Serial.print("Dup:   ");
    Serial.println(gc_status.data2 & 0x08 ? 1:0);
    Serial.print("Ddown: ");
    Serial.println(gc_status.data2 & 0x04 ? 1:0);
    Serial.print("Dright:");
    Serial.println(gc_status.data2 & 0x02 ? 1:0);
    Serial.print("Dleft: ");
    Serial.println(gc_status.data2 & 0x01 ? 1:0);

    Serial.print("Stick X:");
    Serial.println(gc_status.stick_x, DEC);
    Serial.print("Stick Y:");
    Serial.println(gc_status.stick_y, DEC);

    Serial.print("cStick X:");
    Serial.println(gc_status.cstick_x, DEC);
    Serial.print("cStick Y:");
    Serial.println(gc_status.cstick_y, DEC);

    Serial.print("L:     ");
    Serial.println(gc_status.left, DEC);
    Serial.print("R:     ");
    Serial.println(gc_status.right, DEC);
    Serial.flush();
    Serial.write(27);       // ESC command
    Serial.print("[2J");    // clear screen command
    Serial.write(27);
    Serial.print("[H");     // cursor to home command
}

static bool rumble = false;
void loop()
{
    int status;
    unsigned char data, addr;

    // Keep getting fresh data for MegaJoy
    megaJoyControllerData_t controllerData = getControllerData();
    setControllerData(controllerData);

    // clear out incomming raw data buffer
    // this should be unnecessary
    //memset(gc_raw_dump, 0, sizeof(gc_raw_dump));
    //memset(n64_raw_dump, 0, sizeof(n64_raw_dump));

    // Command to send to the gamecube
    // The last bit is rumble, flip it to rumble
    // yes this does need to be inside the loop, the
    // array gets mutilated when it goes through gc_send
    unsigned char command[] = {0x40, 0x03, 0x00};
    if (rumble) {
        command[2] = 0x01;
    }

    // turn on the led, so we can visually see things are happening
    //digitalWrite(13, LOW);
    // don't want interrupts getting in the way
    noInterrupts();
    // send those 3 bytes
    gc_send(command, 3);
    // read in data and dump it to gc_raw_dump
    status = gc_get();
    // end of time sensitive code
    interrupts();
    //digitalWrite(13, HIGH);

    if (status == 0) {
        // problem with getting the gamecube controller status. Maybe it's unplugged?
        // set a neutral N64 string
        // Serial.print(millis(), DEC);
        // Serial.println(" | GC controller read error. Trying to re-initialize");
        // Serial.flush();
        memset(&gc_status, 0, sizeof(gc_status));
        gc_status.stick_x = zero_x;
        gc_status.stick_y = zero_y;
        // this may not work if the controller isn't plugged in, but if it
        // fails we'll try again next loop
        init_gc_controller();
    }

    interrupts();

    // DEBUG: print it
    // print_gc_status();

    // Serial.print(millis(), DEC);
    // Serial.print(" | GC stick: ");
    // Serial.print(gc_status.stick_x, DEC);
    // Serial.print(",");
    // Serial.print(gc_status.stick_y, DEC);
    // Serial.print("  To N64: ");
    // Serial.print(-zero_x + gc_status.stick_x, DEC);
    // Serial.print(",");
    // Serial.println(-zero_y + gc_status.stick_y, DEC);
    // Serial.flush();
  
}

megaJoyControllerData_t getControllerData(void){
  
  // Set up a place for our controller data
  //  Use the getBlankDataForController() function, since
  //  just declaring a fresh dataForController_t tends
  //  to get you one filled with junk from other, random
  //  values that were in those memory locations before
  megaJoyControllerData_t controllerData = getBlankDataForMegaController();

  controllerData.buttonArray[0 / 8] = gc_status.data1 & 0x01 ? 1:0; // A
  controllerData.buttonArray[1 / 8] = gc_status.data1 & 0x02 ? 1:0; // B
  controllerData.buttonArray[2 / 8] = gc_status.data1 & 0x04 ? 1:0; // X
  controllerData.buttonArray[3 / 8] = gc_status.data1 & 0x08 ? 1:0; // Y
  controllerData.buttonArray[4 / 8] = gc_status.data2 & 0x10 ? 1:0; // Z
  controllerData.buttonArray[5 / 8] = gc_status.data2 & 0x40 ? 1:0; // L
  controllerData.buttonArray[6 / 8] = gc_status.data2 & 0x20 ? 1:0; // R
       controllerData.dpad0UpOn = gc_status.data2 & 0x08 ? 1:0;
     controllerData.dpad0DownOn = gc_status.data2 & 0x04 ? 1:0;
     controllerData.dpad0LeftOn = gc_status.data2 & 0x01 ? 1:0;
    controllerData.dpad0RightOn = gc_status.data2 & 0x02 ? 1:0;  




    // Serial.print("Start: ");
    // Serial.println(gc_status.data1 & 0x10 ? 1:0);

    // Serial.print("Y:     ");
    // Serial.println(gc_status.data1 & 0x08 ? 1:0);

    // Serial.print("X:     ");
    // Serial.println(gc_status.data1 & 0x04 ? 1:0);

    // Serial.print("B:     ");
    // Serial.println(gc_status.data1 & 0x02 ? 1:0);

    // Serial.print("A:     ");
    // Serial.println(gc_status.data1 & 0x01 ? 1:0);

    // Serial.print("L:     ");
    // Serial.println(gc_status.data2 & 0x40 ? 1:0);
    // Serial.print("R:     ");
    // Serial.println(gc_status.data2 & 0x20 ? 1:0);
    // Serial.print("Z:     ");
    // Serial.println(gc_status.data2 & 0x10 ? 1:0);

    // Serial.print("Dup:   ");
    // Serial.println(gc_status.data2 & 0x08 ? 1:0);
    // Serial.print("Ddown: ");
    // Serial.println(gc_status.data2 & 0x04 ? 1:0);
    // Serial.print("Dright:");
    // Serial.println(gc_status.data2 & 0x02 ? 1:0);
    // Serial.print("Dleft: ");
    // Serial.println(gc_status.data2 & 0x01 ? 1:0);

    // Serial.print("Stick X:");
    // Serial.println(gc_status.stick_x, DEC);
    // Serial.print("Stick Y:");
    // Serial.println(gc_status.stick_y, DEC);

    // Serial.print("cStick X:");
    // Serial.println(gc_status.cstick_x, DEC);
    // Serial.print("cStick Y:");
    // Serial.println(gc_status.cstick_y, DEC);

    // Serial.print("L:     ");
    // Serial.println(gc_status.left, DEC);
    // Serial.print("R:     ");
    // Serial.println(gc_status.right, DEC);

  
  // Set the analog sticks
  //  Since analogRead(pin) returns a 10 bit value,
  //  we need to perform a bit shift operation to
  //  lose the 2 least significant bits and get an
  //  8 bit number that we can use 
  // controllerData.analogAxisArray[0] = analogRead(A0);
  // controllerData.analogAxisArray[1] = 0;//analogRead(A1); 
  // controllerData.analogAxisArray[2] = 1;//analogRead(A2); 
  // controllerData.analogAxisArray[3] = 512;//analogRead(A3); 
  // controllerData.analogAxisArray[4] = 1023;//analogRead(A4); 
  // controllerData.analogAxisArray[5] = 1000;//analogRead(A5); 
  // controllerData.analogAxisArray[6] = analogRead(A6); 
  // controllerData.analogAxisArray[7] = analogRead(A7); 
  // controllerData.analogAxisArray[8] = analogRead(A8); 
  // controllerData.analogAxisArray[9] = analogRead(A9); 
  // controllerData.analogAxisArray[10] = analogRead(A10); 
  // controllerData.analogAxisArray[11] = analogRead(A11); 
  
  // And return the data!
  return controllerData;
}
