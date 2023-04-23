#include <Pixy2.h>
Pixy2 pixy;

void setup()
{
    Serial.begin(115200);
    pixy.init();
}

void loop()
{
    if (pixy.ccc.numBlocks)
    {
        // print the blocks
        Serial.print("Number of blocks: ");
        Serial.println(pixy.ccc.numBlocks);
        for (int i = 0; i < pixy.ccc.numBlocks; i++)
        {
            pixy.ccc.blocks[i].print();
        }
    }
}