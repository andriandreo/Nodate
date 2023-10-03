/* STM32F10XX BLUEPILL BLINKY - Bare-Metal Firmware Programming from scratch */

void reset(void){
    volatile unsigned int *rcc_apb2enr = (unsigned int *)(0x40021000 + 0x18); // Set address + offset to Reset and Control Clock (RCC) via type cast "(unsigned int *)"
    volatile unsigned int *gpioc_crh   = (unsigned int *)(0x40011000 + 0x04); // Set address + offset to GPIO Configuration Register High (CRH) via = type cast
    volatile unsigned int *gpioc_odr   = (unsigned int *)(0x40011000 + 0x0c); // Set address + offset to GPIO Output Data Register (ODR) via = type cast

    volatile unsigned int i; // Declare counting variable for loop

    /* Enable clock to GPIO port C */
    *rcc_apb2enr |= 1 << 4; // Bits: 1xxxx

    /* Configure the PC13 (user LED) pin for open-drain output, 2MHz max speed */
    *gpioc_crh    = (*gpioc_crh & ~(0xf << ((13-8)*4))) | (6 << ((13-8)*4)); 
        // Bits: 1) *gpioc_crh & ~(111100000000000000000000); interested in bit positions 23, 22 for CRH (start at 0 right - LSB)
        //       2) *gpioc_crh &  (000011111111111111111111) | (011000000000000000000000) 
        //       3) *gpioc_crh &  (011011111111111111111111); bits (23:22)=(01) for open-drain CNF13, (21:20)=(10) for output MODE13 2MHz max speed

    /* Loop Forever */
    while (1){
        /* Wait */
        for (i = 0; i < 500000; i++);
        /* Toggle PC13 */
        *gpioc_odr ^= 1 << 13;
    }
}

int STACK[256]; // Declare an array to hold the STACK pointer/STACK memory (8-bit)

/* Declare an array of 'vectors' of type 'const void *' (pointers) 
and an attibute specifier is applied to instruct the compiler to place it in the ".vectors" section 
of the compiled binary, defined by the linker or compiler configuration */
const void *vectors[] __attribute__ ((section (".vectors"))) = {
    STACK + sizeof(STACK) / sizeof(*STACK), // The address of the last element of the 'STACK' array. 'sizeof()' in bytes, so you divide by that of a single element ('*STACK')
    reset // 'reset' function referring to reset vector, indicating where program execution starts after a reset
}; // The initialiser list sets the elements of the 'vectors' array, just as you would do for "A[2] = {0,1};"