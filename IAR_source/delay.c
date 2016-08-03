void Delay_us(unsigned int Count)
{
   Count *= 12;
   for(; Count!=0; Count--);
}
void Delay_ms(unsigned int Count)
{
   Count *= 12000;
   for(; Count!=0; Count--);
}
void Delay_Spi(unsigned int Count)
{
}