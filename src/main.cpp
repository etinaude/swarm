#include "comms.h"

void setup()
{
  setupEspNow();
}

void loop()
{
  int num = random(10, 20);
  String msg = "test" + String(num);

  broadcast(msg);
  delay(5000);
}