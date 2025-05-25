#include <Wire.h>

#define MAG_ADDR 0x1E

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Init magnétomètre
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x20); Wire.write(0b01100000); Wire.endTransmission();
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x21); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x22); Wire.write(0x00); Wire.endTransmission();

  Serial.println("Tourner le robot à 360° lentement pour calibrer...");
}

void loop() {
  int16_t mx = 0, my = 0;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x28 | 0x80);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 6);

  if (Wire.available() == 6) {
    mx = Wire.read() | (Wire.read() << 8);
    my = Wire.read() | (Wire.read() << 8);
    Wire.read(); Wire.read();

    static int16_t mxMin = 32767, mxMax = -32768;
    static int16_t myMin = 32767, myMax = -32768;

    if (mx < mxMin) mxMin = mx;
    if (mx > mxMax) mxMax = mx;
    if (my < myMin) myMin = my;
    if (my > myMax) myMax = my;

    Serial.print("mx: "); Serial.print(mx);
    Serial.print(" my: "); Serial.print(my);
    Serial.print(" | offsetX = "); Serial.print((mxMin + mxMax) / 2.0);
    Serial.print(" | offsetY = "); Serial.println((myMin + myMax) / 2.0);
  }

  delay(100);
}
