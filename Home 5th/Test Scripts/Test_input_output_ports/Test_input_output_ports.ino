
enum PIN_x
{
  unused_pin_x_0,
  PIN_A,
  PIN_B,
  PIN_C,
  PIN_D,
  PIN_E,
  PIN_F,
  PIN_G,
  PIN_H,
  unused_pin_x_1,
  PIN_J,
  PIN_K,
  PIN_L
} PIN_x_E;

void setup() {
  // put your setup code here, to run once:
// PORTG = 0xff;
//PORTH = PINH | B1111111;  // this is safer as it sets pins 2 to 7 as outputs
 // PORTB = PORTB | B11100000;
// DDRH = 0xFF;
PINH = 0xFF;
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(PINH);
 // PORTH = PORTH | B1111111;
//PORTE = 0xFF;
//  delay(500);
//  PORTH &= ~(1 << 0);
//  PORTH &= ~(1 << 1);
//  PORTH &= ~(1 << 3);
//  PORTH &= ~(1 << 4);
//  PORTH &= ~(1 << 5);
//  PORTH &= ~(1 << 6);
  delay(500);
}
