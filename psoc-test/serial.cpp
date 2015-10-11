//Generates checksum and serial commands. TODO: add to psoc.

void sendserial(unsigned char address, unsigned char command, unsigned char intensity)
{
    if(intensity >127) intensity = 127;
    unsigned char checksum = ((address+command+intensity) & 0b01111111);
    printf("Sending...");
    //Send actual stuff here.
    printf("MOCKSEND ADDRESS: %d\n",address);
    printf("MOCKSEND COMMAND: %d\n",command);
    printf("MOCKSEND INTENSITY: %d\n",intensity);
    printf("MOCKSEND CHECKSUM: %d\n",checksum);
}
