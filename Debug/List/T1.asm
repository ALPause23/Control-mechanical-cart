
;CodeVisionAVR C Compiler V3.12 Advanced
;(C) Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
;http://www.hpinfotech.com

;Build configuration    : Debug
;Chip type              : ATmega8
;Program type           : Boot Loader
;Clock frequency        : 1,000000 MHz
;Memory model           : Small
;Optimize for           : Size
;(s)printf features     : int, width
;(s)scanf features      : int, width
;External RAM size      : 0
;Data Stack size        : 256 byte(s)
;Heap size              : 0 byte(s)
;Promote 'char' to 'int': Yes
;'char' is unsigned     : Yes
;8 bit enums            : Yes
;Global 'const' stored in FLASH: Yes
;Enhanced function parameter passing: Yes
;Enhanced core instructions: On
;Automatic register allocation for global variables: On
;Smart register allocation: On

	#define _MODEL_SMALL_

	#pragma AVRPART ADMIN PART_NAME ATmega8
	#pragma AVRPART MEMORY PROG_FLASH 8192
	#pragma AVRPART MEMORY EEPROM 512
	#pragma AVRPART MEMORY INT_SRAM SIZE 1024
	#pragma AVRPART MEMORY INT_SRAM START_ADDR 0x60

	.LISTMAC
	.EQU UDRE=0x5
	.EQU RXC=0x7
	.EQU USR=0xB
	.EQU UDR=0xC
	.EQU SPSR=0xE
	.EQU SPDR=0xF
	.EQU EERE=0x0
	.EQU EEWE=0x1
	.EQU EEMWE=0x2
	.EQU EECR=0x1C
	.EQU EEDR=0x1D
	.EQU EEARL=0x1E
	.EQU EEARH=0x1F
	.EQU WDTCR=0x21
	.EQU MCUCR=0x35
	.EQU GICR=0x3B
	.EQU SPL=0x3D
	.EQU SPH=0x3E
	.EQU SREG=0x3F

	.DEF R0X0=R0
	.DEF R0X1=R1
	.DEF R0X2=R2
	.DEF R0X3=R3
	.DEF R0X4=R4
	.DEF R0X5=R5
	.DEF R0X6=R6
	.DEF R0X7=R7
	.DEF R0X8=R8
	.DEF R0X9=R9
	.DEF R0XA=R10
	.DEF R0XB=R11
	.DEF R0XC=R12
	.DEF R0XD=R13
	.DEF R0XE=R14
	.DEF R0XF=R15
	.DEF R0X10=R16
	.DEF R0X11=R17
	.DEF R0X12=R18
	.DEF R0X13=R19
	.DEF R0X14=R20
	.DEF R0X15=R21
	.DEF R0X16=R22
	.DEF R0X17=R23
	.DEF R0X18=R24
	.DEF R0X19=R25
	.DEF R0X1A=R26
	.DEF R0X1B=R27
	.DEF R0X1C=R28
	.DEF R0X1D=R29
	.DEF R0X1E=R30
	.DEF R0X1F=R31

	.EQU __SRAM_START=0x0060
	.EQU __SRAM_END=0x045F
	.EQU __DSTACK_SIZE=0x0100
	.EQU __HEAP_SIZE=0x0000
	.EQU __CLEAR_SRAM_SIZE=__SRAM_END-__SRAM_START+1

	.MACRO __CPD1N
	CPI  R30,LOW(@0)
	LDI  R26,HIGH(@0)
	CPC  R31,R26
	LDI  R26,BYTE3(@0)
	CPC  R22,R26
	LDI  R26,BYTE4(@0)
	CPC  R23,R26
	.ENDM

	.MACRO __CPD2N
	CPI  R26,LOW(@0)
	LDI  R30,HIGH(@0)
	CPC  R27,R30
	LDI  R30,BYTE3(@0)
	CPC  R24,R30
	LDI  R30,BYTE4(@0)
	CPC  R25,R30
	.ENDM

	.MACRO __CPWRR
	CP   R@0,R@2
	CPC  R@1,R@3
	.ENDM

	.MACRO __CPWRN
	CPI  R@0,LOW(@2)
	LDI  R30,HIGH(@2)
	CPC  R@1,R30
	.ENDM

	.MACRO __ADDB1MN
	SUBI R30,LOW(-@0-(@1))
	.ENDM

	.MACRO __ADDB2MN
	SUBI R26,LOW(-@0-(@1))
	.ENDM

	.MACRO __ADDW1MN
	SUBI R30,LOW(-@0-(@1))
	SBCI R31,HIGH(-@0-(@1))
	.ENDM

	.MACRO __ADDW2MN
	SUBI R26,LOW(-@0-(@1))
	SBCI R27,HIGH(-@0-(@1))
	.ENDM

	.MACRO __ADDW1FN
	SUBI R30,LOW(-2*@0-(@1))
	SBCI R31,HIGH(-2*@0-(@1))
	.ENDM

	.MACRO __ADDD1FN
	SUBI R30,LOW(-2*@0-(@1))
	SBCI R31,HIGH(-2*@0-(@1))
	SBCI R22,BYTE3(-2*@0-(@1))
	.ENDM

	.MACRO __ADDD1N
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	SBCI R22,BYTE3(-@0)
	SBCI R23,BYTE4(-@0)
	.ENDM

	.MACRO __ADDD2N
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	SBCI R24,BYTE3(-@0)
	SBCI R25,BYTE4(-@0)
	.ENDM

	.MACRO __SUBD1N
	SUBI R30,LOW(@0)
	SBCI R31,HIGH(@0)
	SBCI R22,BYTE3(@0)
	SBCI R23,BYTE4(@0)
	.ENDM

	.MACRO __SUBD2N
	SUBI R26,LOW(@0)
	SBCI R27,HIGH(@0)
	SBCI R24,BYTE3(@0)
	SBCI R25,BYTE4(@0)
	.ENDM

	.MACRO __ANDBMNN
	LDS  R30,@0+(@1)
	ANDI R30,LOW(@2)
	STS  @0+(@1),R30
	.ENDM

	.MACRO __ANDWMNN
	LDS  R30,@0+(@1)
	ANDI R30,LOW(@2)
	STS  @0+(@1),R30
	LDS  R30,@0+(@1)+1
	ANDI R30,HIGH(@2)
	STS  @0+(@1)+1,R30
	.ENDM

	.MACRO __ANDD1N
	ANDI R30,LOW(@0)
	ANDI R31,HIGH(@0)
	ANDI R22,BYTE3(@0)
	ANDI R23,BYTE4(@0)
	.ENDM

	.MACRO __ANDD2N
	ANDI R26,LOW(@0)
	ANDI R27,HIGH(@0)
	ANDI R24,BYTE3(@0)
	ANDI R25,BYTE4(@0)
	.ENDM

	.MACRO __ORBMNN
	LDS  R30,@0+(@1)
	ORI  R30,LOW(@2)
	STS  @0+(@1),R30
	.ENDM

	.MACRO __ORWMNN
	LDS  R30,@0+(@1)
	ORI  R30,LOW(@2)
	STS  @0+(@1),R30
	LDS  R30,@0+(@1)+1
	ORI  R30,HIGH(@2)
	STS  @0+(@1)+1,R30
	.ENDM

	.MACRO __ORD1N
	ORI  R30,LOW(@0)
	ORI  R31,HIGH(@0)
	ORI  R22,BYTE3(@0)
	ORI  R23,BYTE4(@0)
	.ENDM

	.MACRO __ORD2N
	ORI  R26,LOW(@0)
	ORI  R27,HIGH(@0)
	ORI  R24,BYTE3(@0)
	ORI  R25,BYTE4(@0)
	.ENDM

	.MACRO __DELAY_USB
	LDI  R24,LOW(@0)
__DELAY_USB_LOOP:
	DEC  R24
	BRNE __DELAY_USB_LOOP
	.ENDM

	.MACRO __DELAY_USW
	LDI  R24,LOW(@0)
	LDI  R25,HIGH(@0)
__DELAY_USW_LOOP:
	SBIW R24,1
	BRNE __DELAY_USW_LOOP
	.ENDM

	.MACRO __GETD1S
	LDD  R30,Y+@0
	LDD  R31,Y+@0+1
	LDD  R22,Y+@0+2
	LDD  R23,Y+@0+3
	.ENDM

	.MACRO __GETD2S
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	LDD  R24,Y+@0+2
	LDD  R25,Y+@0+3
	.ENDM

	.MACRO __PUTD1S
	STD  Y+@0,R30
	STD  Y+@0+1,R31
	STD  Y+@0+2,R22
	STD  Y+@0+3,R23
	.ENDM

	.MACRO __PUTD2S
	STD  Y+@0,R26
	STD  Y+@0+1,R27
	STD  Y+@0+2,R24
	STD  Y+@0+3,R25
	.ENDM

	.MACRO __PUTDZ2
	STD  Z+@0,R26
	STD  Z+@0+1,R27
	STD  Z+@0+2,R24
	STD  Z+@0+3,R25
	.ENDM

	.MACRO __CLRD1S
	STD  Y+@0,R30
	STD  Y+@0+1,R30
	STD  Y+@0+2,R30
	STD  Y+@0+3,R30
	.ENDM

	.MACRO __POINTB1MN
	LDI  R30,LOW(@0+(@1))
	.ENDM

	.MACRO __POINTW1MN
	LDI  R30,LOW(@0+(@1))
	LDI  R31,HIGH(@0+(@1))
	.ENDM

	.MACRO __POINTD1M
	LDI  R30,LOW(@0)
	LDI  R31,HIGH(@0)
	LDI  R22,BYTE3(@0)
	LDI  R23,BYTE4(@0)
	.ENDM

	.MACRO __POINTW1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	.ENDM

	.MACRO __POINTD1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	LDI  R22,BYTE3(2*@0+(@1))
	LDI  R23,BYTE4(2*@0+(@1))
	.ENDM

	.MACRO __POINTB2MN
	LDI  R26,LOW(@0+(@1))
	.ENDM

	.MACRO __POINTW2MN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	.ENDM

	.MACRO __POINTW2FN
	LDI  R26,LOW(2*@0+(@1))
	LDI  R27,HIGH(2*@0+(@1))
	.ENDM

	.MACRO __POINTD2FN
	LDI  R26,LOW(2*@0+(@1))
	LDI  R27,HIGH(2*@0+(@1))
	LDI  R24,BYTE3(2*@0+(@1))
	LDI  R25,BYTE4(2*@0+(@1))
	.ENDM

	.MACRO __POINTBRM
	LDI  R@0,LOW(@1)
	.ENDM

	.MACRO __POINTWRM
	LDI  R@0,LOW(@2)
	LDI  R@1,HIGH(@2)
	.ENDM

	.MACRO __POINTBRMN
	LDI  R@0,LOW(@1+(@2))
	.ENDM

	.MACRO __POINTWRMN
	LDI  R@0,LOW(@2+(@3))
	LDI  R@1,HIGH(@2+(@3))
	.ENDM

	.MACRO __POINTWRFN
	LDI  R@0,LOW(@2*2+(@3))
	LDI  R@1,HIGH(@2*2+(@3))
	.ENDM

	.MACRO __GETD1N
	LDI  R30,LOW(@0)
	LDI  R31,HIGH(@0)
	LDI  R22,BYTE3(@0)
	LDI  R23,BYTE4(@0)
	.ENDM

	.MACRO __GETD2N
	LDI  R26,LOW(@0)
	LDI  R27,HIGH(@0)
	LDI  R24,BYTE3(@0)
	LDI  R25,BYTE4(@0)
	.ENDM

	.MACRO __GETB1MN
	LDS  R30,@0+(@1)
	.ENDM

	.MACRO __GETB1HMN
	LDS  R31,@0+(@1)
	.ENDM

	.MACRO __GETW1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	.ENDM

	.MACRO __GETD1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	LDS  R22,@0+(@1)+2
	LDS  R23,@0+(@1)+3
	.ENDM

	.MACRO __GETBRMN
	LDS  R@0,@1+(@2)
	.ENDM

	.MACRO __GETWRMN
	LDS  R@0,@2+(@3)
	LDS  R@1,@2+(@3)+1
	.ENDM

	.MACRO __GETWRZ
	LDD  R@0,Z+@2
	LDD  R@1,Z+@2+1
	.ENDM

	.MACRO __GETD2Z
	LDD  R26,Z+@0
	LDD  R27,Z+@0+1
	LDD  R24,Z+@0+2
	LDD  R25,Z+@0+3
	.ENDM

	.MACRO __GETB2MN
	LDS  R26,@0+(@1)
	.ENDM

	.MACRO __GETW2MN
	LDS  R26,@0+(@1)
	LDS  R27,@0+(@1)+1
	.ENDM

	.MACRO __GETD2MN
	LDS  R26,@0+(@1)
	LDS  R27,@0+(@1)+1
	LDS  R24,@0+(@1)+2
	LDS  R25,@0+(@1)+3
	.ENDM

	.MACRO __PUTB1MN
	STS  @0+(@1),R30
	.ENDM

	.MACRO __PUTW1MN
	STS  @0+(@1),R30
	STS  @0+(@1)+1,R31
	.ENDM

	.MACRO __PUTD1MN
	STS  @0+(@1),R30
	STS  @0+(@1)+1,R31
	STS  @0+(@1)+2,R22
	STS  @0+(@1)+3,R23
	.ENDM

	.MACRO __PUTB1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMWRB
	.ENDM

	.MACRO __PUTW1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMWRW
	.ENDM

	.MACRO __PUTD1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMWRD
	.ENDM

	.MACRO __PUTBR0MN
	STS  @0+(@1),R0
	.ENDM

	.MACRO __PUTBMRN
	STS  @0+(@1),R@2
	.ENDM

	.MACRO __PUTWMRN
	STS  @0+(@1),R@2
	STS  @0+(@1)+1,R@3
	.ENDM

	.MACRO __PUTBZR
	STD  Z+@1,R@0
	.ENDM

	.MACRO __PUTWZR
	STD  Z+@2,R@0
	STD  Z+@2+1,R@1
	.ENDM

	.MACRO __GETW1R
	MOV  R30,R@0
	MOV  R31,R@1
	.ENDM

	.MACRO __GETW2R
	MOV  R26,R@0
	MOV  R27,R@1
	.ENDM

	.MACRO __GETWRN
	LDI  R@0,LOW(@2)
	LDI  R@1,HIGH(@2)
	.ENDM

	.MACRO __PUTW1R
	MOV  R@0,R30
	MOV  R@1,R31
	.ENDM

	.MACRO __PUTW2R
	MOV  R@0,R26
	MOV  R@1,R27
	.ENDM

	.MACRO __ADDWRN
	SUBI R@0,LOW(-@2)
	SBCI R@1,HIGH(-@2)
	.ENDM

	.MACRO __ADDWRR
	ADD  R@0,R@2
	ADC  R@1,R@3
	.ENDM

	.MACRO __SUBWRN
	SUBI R@0,LOW(@2)
	SBCI R@1,HIGH(@2)
	.ENDM

	.MACRO __SUBWRR
	SUB  R@0,R@2
	SBC  R@1,R@3
	.ENDM

	.MACRO __ANDWRN
	ANDI R@0,LOW(@2)
	ANDI R@1,HIGH(@2)
	.ENDM

	.MACRO __ANDWRR
	AND  R@0,R@2
	AND  R@1,R@3
	.ENDM

	.MACRO __ORWRN
	ORI  R@0,LOW(@2)
	ORI  R@1,HIGH(@2)
	.ENDM

	.MACRO __ORWRR
	OR   R@0,R@2
	OR   R@1,R@3
	.ENDM

	.MACRO __EORWRR
	EOR  R@0,R@2
	EOR  R@1,R@3
	.ENDM

	.MACRO __GETWRS
	LDD  R@0,Y+@2
	LDD  R@1,Y+@2+1
	.ENDM

	.MACRO __PUTBSR
	STD  Y+@1,R@0
	.ENDM

	.MACRO __PUTWSR
	STD  Y+@2,R@0
	STD  Y+@2+1,R@1
	.ENDM

	.MACRO __MOVEWRR
	MOV  R@0,R@2
	MOV  R@1,R@3
	.ENDM

	.MACRO __INWR
	IN   R@0,@2
	IN   R@1,@2+1
	.ENDM

	.MACRO __OUTWR
	OUT  @2+1,R@1
	OUT  @2,R@0
	.ENDM

	.MACRO __CALL1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	ICALL
	.ENDM

	.MACRO __CALL1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	RCALL __GETW1PF
	ICALL
	.ENDM

	.MACRO __CALL2EN
	PUSH R26
	PUSH R27
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMRDW
	POP  R27
	POP  R26
	ICALL
	.ENDM

	.MACRO __CALL2EX
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	RCALL __EEPROMRDD
	ICALL
	.ENDM

	.MACRO __GETW1STACK
	IN   R30,SPL
	IN   R31,SPH
	ADIW R30,@0+1
	LD   R0,Z+
	LD   R31,Z
	MOV  R30,R0
	.ENDM

	.MACRO __GETD1STACK
	IN   R30,SPL
	IN   R31,SPH
	ADIW R30,@0+1
	LD   R0,Z+
	LD   R1,Z+
	LD   R22,Z
	MOVW R30,R0
	.ENDM

	.MACRO __NBST
	BST  R@0,@1
	IN   R30,SREG
	LDI  R31,0x40
	EOR  R30,R31
	OUT  SREG,R30
	.ENDM


	.MACRO __PUTB1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RNS
	MOVW R26,R@0
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RNS
	MOVW R26,R@0
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RNS
	MOVW R26,R@0
	ADIW R26,@1
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	RCALL __PUTDP1
	.ENDM


	.MACRO __GETB1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R30,Z
	.ENDM

	.MACRO __GETB1HSX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R31,Z
	.ENDM

	.MACRO __GETW1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R0,Z+
	LD   R31,Z
	MOV  R30,R0
	.ENDM

	.MACRO __GETD1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R0,Z+
	LD   R1,Z+
	LD   R22,Z+
	LD   R23,Z
	MOVW R30,R0
	.ENDM

	.MACRO __GETB2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R26,X
	.ENDM

	.MACRO __GETW2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	.ENDM

	.MACRO __GETD2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R1,X+
	LD   R24,X+
	LD   R25,X
	MOVW R26,R0
	.ENDM

	.MACRO __GETBRSX
	MOVW R30,R28
	SUBI R30,LOW(-@1)
	SBCI R31,HIGH(-@1)
	LD   R@0,Z
	.ENDM

	.MACRO __GETWRSX
	MOVW R30,R28
	SUBI R30,LOW(-@2)
	SBCI R31,HIGH(-@2)
	LD   R@0,Z+
	LD   R@1,Z
	.ENDM

	.MACRO __GETBRSX2
	MOVW R26,R28
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	LD   R@0,X
	.ENDM

	.MACRO __GETWRSX2
	MOVW R26,R28
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	LD   R@0,X+
	LD   R@1,X
	.ENDM

	.MACRO __LSLW8SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R31,Z
	CLR  R30
	.ENDM

	.MACRO __PUTB1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	.ENDM

	.MACRO __CLRW1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X,R30
	.ENDM

	.MACRO __CLRD1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X+,R30
	ST   X+,R30
	ST   X,R30
	.ENDM

	.MACRO __PUTB2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z,R26
	.ENDM

	.MACRO __PUTW2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z+,R26
	ST   Z,R27
	.ENDM

	.MACRO __PUTD2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z+,R26
	ST   Z+,R27
	ST   Z+,R24
	ST   Z,R25
	.ENDM

	.MACRO __PUTBSRX
	MOVW R30,R28
	SUBI R30,LOW(-@1)
	SBCI R31,HIGH(-@1)
	ST   Z,R@0
	.ENDM

	.MACRO __PUTWSRX
	MOVW R30,R28
	SUBI R30,LOW(-@2)
	SBCI R31,HIGH(-@2)
	ST   Z+,R@0
	ST   Z,R@1
	.ENDM

	.MACRO __PUTB1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	.ENDM

	.MACRO __MULBRR
	MULS R@0,R@1
	MOVW R30,R0
	.ENDM

	.MACRO __MULBRRU
	MUL  R@0,R@1
	MOVW R30,R0
	.ENDM

	.MACRO __MULBRR0
	MULS R@0,R@1
	.ENDM

	.MACRO __MULBRRU0
	MUL  R@0,R@1
	.ENDM

	.MACRO __MULBNWRU
	LDI  R26,@2
	MUL  R26,R@0
	MOVW R30,R0
	MUL  R26,R@1
	ADD  R31,R0
	.ENDM

;NAME DEFINITIONS FOR GLOBAL VARIABLES ALLOCATED TO REGISTERS
	.DEF _PWMDir=R5
	.DEF _statusMotor=R4
	.DEF _numButtom=R6
	.DEF _numButtom_msb=R7
	.DEF _tampUpStop=R9
	.DEF _tampDownStop=R8

	.CSEG
	.ORG 0xC00

;START OF CODE MARKER
__START_OF_CODE:

;INTERRUPT VECTORS
	RJMP __RESET
	RJMP _exterInt0
	RJMP _exterInt1
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00
	RJMP 0xC00

;GLOBAL REGISTER VARIABLES INITIALIZATION
__REG_VARS:
	.DB  0x0,0x0,0x0,0x0
	.DB  0x0,0x0


__GLOBAL_INI_TBL:
	.DW  0x06
	.DW  0x04
	.DW  __REG_VARS*2

_0xFFFFFFFF:
	.DW  0

#define __GLOBAL_INI_TBL_PRESENT 1

__RESET:
	CLI
	CLR  R30
	OUT  EECR,R30

;INTERRUPT VECTORS ARE PLACED
;AT THE START OF THE BOOT LOADER
	LDI  R31,1
	OUT  GICR,R31
	LDI  R31,2
	OUT  GICR,R31
	OUT  MCUCR,R30

;CLEAR R2-R14
	LDI  R24,(14-2)+1
	LDI  R26,2
	CLR  R27
__CLEAR_REG:
	ST   X+,R30
	DEC  R24
	BRNE __CLEAR_REG

;CLEAR SRAM
	LDI  R24,LOW(__CLEAR_SRAM_SIZE)
	LDI  R25,HIGH(__CLEAR_SRAM_SIZE)
	LDI  R26,__SRAM_START
__CLEAR_SRAM:
	ST   X+,R30
	SBIW R24,1
	BRNE __CLEAR_SRAM

;GLOBAL VARIABLES INITIALIZATION
	LDI  R30,LOW(__GLOBAL_INI_TBL*2)
	LDI  R31,HIGH(__GLOBAL_INI_TBL*2)
__GLOBAL_INI_NEXT:
	LPM  R24,Z+
	LPM  R25,Z+
	SBIW R24,0
	BREQ __GLOBAL_INI_END
	LPM  R26,Z+
	LPM  R27,Z+
	LPM  R0,Z+
	LPM  R1,Z+
	MOVW R22,R30
	MOVW R30,R0
__GLOBAL_INI_LOOP:
	LPM  R0,Z+
	ST   X+,R0
	SBIW R24,1
	BRNE __GLOBAL_INI_LOOP
	MOVW R30,R22
	RJMP __GLOBAL_INI_NEXT
__GLOBAL_INI_END:

;HARDWARE STACK POINTER INITIALIZATION
	LDI  R30,LOW(__SRAM_END-__HEAP_SIZE)
	OUT  SPL,R30
	LDI  R30,HIGH(__SRAM_END-__HEAP_SIZE)
	OUT  SPH,R30

;DATA STACK POINTER INITIALIZATION
	LDI  R28,LOW(__SRAM_START+__DSTACK_SIZE)
	LDI  R29,HIGH(__SRAM_START+__DSTACK_SIZE)

	RJMP _main

	.ESEG
	.ORG 0

	.DSEG
	.ORG 0x160

	.CSEG
;#define F_CPU 1000000UL
;#include <mega8.h>
	#ifndef __SLEEP_DEFINED__
	#define __SLEEP_DEFINED__
	.EQU __se_bit=0x80
	.EQU __sm_mask=0x70
	.EQU __sm_powerdown=0x20
	.EQU __sm_powersave=0x30
	.EQU __sm_standby=0x60
	.EQU __sm_ext_standby=0x70
	.EQU __sm_adc_noise_red=0x10
	.SET power_ctrl_reg=mcucr
	#endif
;#include <io.h>
;#include <delay.h>
;#include <stdbool.h>
;
;#define PC0   (1<<0); // �����
;#define PC1   (1<<1); // �����
;#define PC2   (1<<2); // ������ �������
;#define PC3   (1<<3); // ������ �����
;#define PC4   (1<<4); // ����������� �� ������������
;
;#define PB0   (1<<0); // ������� ����� ��������
;#define PB1   (1<<1); // �������  ����� ��������
;#define PB3   (1<<3); // PWM
;#define PB4   (1<<4); // direction
;#define PB5   (1<<5); // ������ ������� (up)
;#define PB6   (1<<6); // ������ ������� (down)
;#define PB7   (1<<7); // �����������
;
;#define PD0   (1<<0);
;#define PD1   (1<<1);
;#define PD4   (1<<4);
;#define PD5   (1<<5);
;#define PD6   (1<<6);
;#define PD7   (1<<7);
;
;bool PWMDir = 0;  // 0 - start, 1 - stop
;bool statusMotor = 0;  // 1 - worked, 0 - stopped
;unsigned int numButtom;  // 0 - up, 1 - down
;bool tampUpStop = false;
;bool tampDownStop = false;
;
;void initializationDefolt()
; 0000 0023 {

	.CSEG
_initializationDefolt:
; .FSTART _initializationDefolt
; 0000 0024     // Declare your local variables here
; 0000 0025 
; 0000 0026     // Input/Output Ports initialization
; 0000 0027     // Port B initialization
; 0000 0028     // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=OUT Bit2=In Bit1=In Bit0=In
; 0000 0029     DDRB=(1<<DDB7) | (1<<DDB6) | (1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (0<<DDB2) | (1<<DDB1) | (1<<DDB0);
	LDI  R30,LOW(251)
	OUT  0x17,R30
; 0000 002A     // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
; 0000 002B     PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);
	LDI  R30,LOW(0)
	OUT  0x18,R30
; 0000 002C 
; 0000 002D     // Port C initialization
; 0000 002E     // Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
; 0000 002F     DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (1<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
	LDI  R30,LOW(8)
	OUT  0x14,R30
; 0000 0030     // State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
; 0000 0031     PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);
	LDI  R30,LOW(0)
	OUT  0x15,R30
; 0000 0032 
; 0000 0033     // Port D initialization
; 0000 0034     // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
; 0000 0035     DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (1<<DDD1) | (1<<DDD0);
	LDI  R30,LOW(243)
	OUT  0x11,R30
; 0000 0036     // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
; 0000 0037     PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);
	LDI  R30,LOW(0)
	OUT  0x12,R30
; 0000 0038 
; 0000 0039     // Timer/Counter 0 initialization
; 0000 003A     // Clock source: System Clock
; 0000 003B     // Clock value: Timer 0 Stopped
; 0000 003C     TCCR0=(0<<CS02) | (0<<CS01) | (0<<CS00);
	OUT  0x33,R30
; 0000 003D     TCNT0=0x00;
	OUT  0x32,R30
; 0000 003E 
; 0000 003F     // Timer/Counter 1 initialization
; 0000 0040     // Clock source: System Clock
; 0000 0041     // Clock value: Timer1 Stopped
; 0000 0042     // Mode: Normal top=0xFFFF
; 0000 0043     // OC1A output: Disconnected
; 0000 0044     // OC1B output: Disconnected
; 0000 0045     // Noise Canceler: Off
; 0000 0046     // Input Capture on Falling Edge
; 0000 0047     // Timer1 Overflow Interrupt: Off
; 0000 0048     // Input Capture Interrupt: Off
; 0000 0049     // Compare A Match Interrupt: Off
; 0000 004A     // Compare B Match Interrupt: Off
; 0000 004B     TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
	OUT  0x2F,R30
; 0000 004C     TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
	OUT  0x2E,R30
; 0000 004D     TCNT1H=0x00;
	OUT  0x2D,R30
; 0000 004E     TCNT1L=0x00;
	OUT  0x2C,R30
; 0000 004F     ICR1H=0x00;
	OUT  0x27,R30
; 0000 0050     ICR1L=0x00;
	OUT  0x26,R30
; 0000 0051     OCR1AH=0x00;
	OUT  0x2B,R30
; 0000 0052     OCR1AL=0x00;
	OUT  0x2A,R30
; 0000 0053     OCR1BH=0x00;
	OUT  0x29,R30
; 0000 0054     OCR1BL=0x00;
	OUT  0x28,R30
; 0000 0055 
; 0000 0056     // Timer/Counter 2 initialization
; 0000 0057     // Clock source: System Clock
; 0000 0058     // Clock value: Timer2 Stopped
; 0000 0059     // Mode: Normal top=0xFF
; 0000 005A     // OC2 output: Disconnected
; 0000 005B     ASSR=0<<AS2;
	RCALL SUBOPT_0x0
; 0000 005C     TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (0<<CS21) | (0<<CS20);
; 0000 005D     TCNT2=0x00;
; 0000 005E     OCR2=0x00;
; 0000 005F 
; 0000 0060     // Timer(s)/Counter(s) Interrupt(s) initialization
; 0000 0061     TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<TOIE0);
	LDI  R30,LOW(0)
	OUT  0x39,R30
; 0000 0062 
; 0000 0063     // External Interrupt(s) initialization
; 0000 0064     // INT0: Off
; 0000 0065     // INT1: Off
; 0000 0066     GICR|=(1<<INT1) | (1<<INT0);
	IN   R30,0x3B
	ORI  R30,LOW(0xC0)
	OUT  0x3B,R30
; 0000 0067     MCUCR=(0<<ISC11) | (1<<ISC10) | (1<<ISC01) | (0<<ISC00);
	LDI  R30,LOW(6)
	OUT  0x35,R30
; 0000 0068     GIFR=(0<<INTF1) | (0<<INTF0);
	LDI  R30,LOW(0)
	OUT  0x3A,R30
; 0000 0069 
; 0000 006A     // USART initialization
; 0000 006B     // USART disabled
; 0000 006C     UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
	OUT  0xA,R30
; 0000 006D 
; 0000 006E     // Analog Comparator initialization
; 0000 006F     // Analog Comparator: Off
; 0000 0070     // The Analog Comparator's positive input is
; 0000 0071     // connected to the AIN0 pin
; 0000 0072     // The Analog Comparator's negative input is
; 0000 0073     // connected to the AIN1 pin
; 0000 0074     ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
	LDI  R30,LOW(128)
	OUT  0x8,R30
; 0000 0075     SFIOR=(0<<ACME);
	LDI  R30,LOW(0)
	OUT  0x30,R30
; 0000 0076 
; 0000 0077     // ADC initialization
; 0000 0078     // ADC disabled
; 0000 0079     ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
	OUT  0x6,R30
; 0000 007A 
; 0000 007B     // SPI initialization
; 0000 007C     // SPI disabled
; 0000 007D     SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);
	OUT  0xD,R30
; 0000 007E 
; 0000 007F     // TWI initialization
; 0000 0080     // TWI disabled
; 0000 0081     TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);
	OUT  0x36,R30
; 0000 0082 }
	RET
; .FEND
;
;void initPWM()
; 0000 0085 {
_initPWM:
; .FSTART _initPWM
; 0000 0086     ASSR = 0x00;
	RCALL SUBOPT_0x0
; 0000 0087     TCCR2 = 0x00;    // ������ ��������
; 0000 0088     TCNT2 = 0x00;
; 0000 0089     OCR2 = 0x00;
; 0000 008A }
	RET
; .FEND
;
;void ledArrowControl(char dir, bool onOff)
; 0000 008D {
_ledArrowControl:
; .FSTART _ledArrowControl
; 0000 008E     switch(dir)
	ST   -Y,R26
;	dir -> Y+1
;	onOff -> Y+0
	LDD  R30,Y+1
	LDI  R31,0
; 0000 008F     {
; 0000 0090         case 'u': if (onOff) PORTB |= PB0;
	CPI  R30,LOW(0x75)
	LDI  R26,HIGH(0x75)
	CPC  R31,R26
	BRNE _0x6
	LD   R30,Y
	CPI  R30,0
	BREQ _0x7
	SBI  0x18,0
; 0000 0091                 else PORTB &= ~((1<<0)|(1<<1));
	RJMP _0x8
_0x7:
	IN   R30,0x18
	ANDI R30,LOW(0xFC)
	OUT  0x18,R30
; 0000 0092                 break;
_0x8:
	RJMP _0x5
; 0000 0093         case 'd': if (onOff) PORTB |= PB1;
_0x6:
	CPI  R30,LOW(0x64)
	LDI  R26,HIGH(0x64)
	CPC  R31,R26
	BRNE _0x5
	LD   R30,Y
	CPI  R30,0
	BREQ _0xA
	SBI  0x18,1
; 0000 0094                 else PORTB &= ~((1<<0)|(1<<1));
	RJMP _0xB
_0xA:
	IN   R30,0x18
	ANDI R30,LOW(0xFC)
	OUT  0x18,R30
; 0000 0095                 break;
_0xB:
; 0000 0096     }
_0x5:
; 0000 0097 }
	ADIW R28,2
	RET
; .FEND
;
;unsigned int checkPortPC(int i, int n)
; 0000 009A {
_checkPortPC:
; .FSTART _checkPortPC
; 0000 009B 
; 0000 009C     int j = 0, k = 0;
; 0000 009D 
; 0000 009E     for(i; i <= n; i++)
	ST   -Y,R27
	ST   -Y,R26
	RCALL __SAVELOCR4
;	i -> Y+6
;	n -> Y+4
;	j -> R16,R17
;	k -> R18,R19
	__GETWRN 16,17,0
	__GETWRN 18,19,0
	LDD  R30,Y+6
	LDD  R31,Y+6+1
_0xD:
	LDD  R30,Y+4
	LDD  R31,Y+4+1
	LDD  R26,Y+6
	LDD  R27,Y+6+1
	CP   R30,R26
	CPC  R31,R27
	BRLT _0xE
; 0000 009F     {
; 0000 00A0         if(~PINC & (1<<i))
	IN   R30,0x13
	COM  R30
	MOV  R1,R30
	LDD  R30,Y+6
	LDI  R26,LOW(1)
	LDI  R27,HIGH(1)
	RCALL __LSLW12
	MOV  R26,R1
	LDI  R27,0
	AND  R30,R26
	AND  R31,R27
	SBIW R30,0
	BREQ _0xF
; 0000 00A1         {
; 0000 00A2             PORTD |= (1<<i);
	RCALL SUBOPT_0x1
	OR   R30,R1
	OUT  0x12,R30
; 0000 00A3             j = i;
	__GETWRS 16,17,6
; 0000 00A4             k++;
	__ADDWRN 18,19,1
; 0000 00A5             PORTD &= ~(1<<i);
	RCALL SUBOPT_0x1
	COM  R30
	AND  R30,R1
	OUT  0x12,R30
; 0000 00A6         }
; 0000 00A7     }
_0xF:
	LDD  R30,Y+6
	LDD  R31,Y+6+1
	ADIW R30,1
	STD  Y+6,R30
	STD  Y+6+1,R31
	RJMP _0xD
_0xE:
; 0000 00A8     if(k == 1)
	LDI  R30,LOW(1)
	LDI  R31,HIGH(1)
	CP   R30,R18
	CPC  R31,R19
	BRNE _0x10
; 0000 00A9     {
; 0000 00AA 
; 0000 00AB         return j;
	MOVW R30,R16
	RJMP _0x2000002
; 0000 00AC     }
; 0000 00AD     else {PORTD |= PD4;
_0x10:
	SBI  0x12,4
; 0000 00AE        PORTD &= ~PD4;return 255;}
	CBI  0x12,4
	LDI  R30,LOW(255)
	LDI  R31,HIGH(255)
; 0000 00AF }
_0x2000002:
	RCALL __LOADLOCR4
	ADIW R28,8
	RET
; .FEND
;
;void PWM(char dir)
; 0000 00B2 {
_PWM:
; .FSTART _PWM
; 0000 00B3 
; 0000 00B4     if(PWMDir == 0)
	ST   -Y,R26
;	dir -> Y+0
	TST  R5
	BRNE _0x12
; 0000 00B5     {
; 0000 00B6         OCR2 = 0x66;
	LDI  R30,LOW(102)
	RCALL SUBOPT_0x2
; 0000 00B7         TCCR2 = 0b01101100; //start timer
; 0000 00B8         ledArrowControl(dir, 1);
	LDI  R26,LOW(1)
	RCALL _ledArrowControl
; 0000 00B9         while(OCR2 != 0xFF)
_0x13:
	IN   R30,0x23
	CPI  R30,LOW(0xFF)
	BREQ _0x15
; 0000 00BA         {
; 0000 00BB             OCR2++;
	IN   R30,0x23
	SUBI R30,-LOW(1)
	RCALL SUBOPT_0x3
; 0000 00BC             delay_ms(7);
; 0000 00BD         }
	RJMP _0x13
_0x15:
; 0000 00BE         PWMDir = 1;
	LDI  R30,LOW(1)
	MOV  R5,R30
; 0000 00BF     }
; 0000 00C0     else
	RJMP _0x16
_0x12:
; 0000 00C1     {
; 0000 00C2         OCR2 = 0xFF;
	LDI  R30,LOW(255)
	RCALL SUBOPT_0x2
; 0000 00C3         TCCR2 = 0b01101100; //start timer
; 0000 00C4         ledArrowControl(dir, 0);
	RCALL SUBOPT_0x4
; 0000 00C5         while(OCR2 != 0x66)
_0x17:
	IN   R30,0x23
	CPI  R30,LOW(0x66)
	BREQ _0x19
; 0000 00C6         {
; 0000 00C7             OCR2--;
	IN   R30,0x23
	SUBI R30,LOW(1)
	RCALL SUBOPT_0x3
; 0000 00C8             delay_ms(7);
; 0000 00C9         }
	RJMP _0x17
_0x19:
; 0000 00CA         PWMDir = 0;
	CLR  R5
; 0000 00CB     }
_0x16:
; 0000 00CC     TCCR2 = 0x00; //stop timer
	LDI  R30,LOW(0)
	OUT  0x25,R30
; 0000 00CD     OCR2 = 0x00;
	OUT  0x23,R30
; 0000 00CE     if(PWMDir) { PORTB |= PB3;}
	TST  R5
	BREQ _0x1A
	SBI  0x18,3
; 0000 00CF     else PORTB &= ~PB3;
	RJMP _0x1B
_0x1A:
	CBI  0x18,3
; 0000 00D0     return;
_0x1B:
	RJMP _0x2000001
; 0000 00D1 }
; .FEND
;
;void goUpDown(char dir) //1 - up, 0 - down
; 0000 00D4 {
_goUpDown:
; .FSTART _goUpDown
; 0000 00D5    if (dir)
	ST   -Y,R26
;	dir -> Y+0
	LD   R30,Y
	CPI  R30,0
	BREQ _0x1C
; 0000 00D6     {
; 0000 00D7         PORTB |= PB4;
	SBI  0x18,4
; 0000 00D8         PWM('u');
	LDI  R26,LOW(117)
	RJMP _0x39
; 0000 00D9     }
; 0000 00DA    else
_0x1C:
; 0000 00DB    {
; 0000 00DC         PORTB &= ~PB4;
	CBI  0x18,4
; 0000 00DD         PWM('d');
	LDI  R26,LOW(100)
_0x39:
	RCALL _PWM
; 0000 00DE    }
; 0000 00DF    if(statusMotor == 0)
	TST  R4
	BRNE _0x1E
; 0000 00E0    statusMotor = 1;
	LDI  R30,LOW(1)
	MOV  R4,R30
; 0000 00E1    else statusMotor = 0;
	RJMP _0x1F
_0x1E:
	CLR  R4
; 0000 00E2 }
_0x1F:
_0x2000001:
	ADIW R28,1
	RET
; .FEND
;
;void main()
; 0000 00E5 {
_main:
; .FSTART _main
; 0000 00E6     #asm("cli")
	cli
; 0000 00E7     initializationDefolt();
	RCALL _initializationDefolt
; 0000 00E8     initPWM();
	RCALL _initPWM
; 0000 00E9     #asm("sei")
	sei
; 0000 00EA     while (1)
_0x20:
; 0000 00EB     {
; 0000 00EC        if(PINC & (1<<4))
	SBIS 0x13,4
	RJMP _0x23
; 0000 00ED        {
; 0000 00EE             PORTB |= PB7;
	SBI  0x18,7
; 0000 00EF             tampUpStop = false;
	CLR  R9
; 0000 00F0             tampDownStop = false;
	CLR  R8
; 0000 00F1             PORTB &= ~PB3;
	CBI  0x18,3
; 0000 00F2             statusMotor = 0;
	CLR  R4
; 0000 00F3             PWMDir = 0;
	CLR  R5
; 0000 00F4             ledArrowControl('u', false);
	LDI  R30,LOW(117)
	ST   -Y,R30
	RCALL SUBOPT_0x4
; 0000 00F5        }
; 0000 00F6        else PORTB &= ~PB7;
	RJMP _0x24
_0x23:
	CBI  0x18,7
; 0000 00F7     }
_0x24:
	RJMP _0x20
; 0000 00F8 }
_0x25:
	RJMP _0x25
; .FEND
;
;interrupt [EXT_INT0] void exterInt0(void)
; 0000 00FB {
_exterInt0:
; .FSTART _exterInt0
	RCALL SUBOPT_0x5
; 0000 00FC     if (statusMotor)
	BREQ _0x26
; 0000 00FD     {
; 0000 00FE        switch(checkPortPC(2, 3))
	LDI  R30,LOW(2)
	LDI  R31,HIGH(2)
	ST   -Y,R31
	ST   -Y,R30
	LDI  R26,LOW(3)
	LDI  R27,0
	RCALL _checkPortPC
; 0000 00FF        {
; 0000 0100            case 2:PORTB |= PB5;
	CPI  R30,LOW(0x2)
	LDI  R26,HIGH(0x2)
	CPC  R31,R26
	BRNE _0x2A
	SBI  0x18,5
; 0000 0101                statusMotor = 0;
	CLR  R4
; 0000 0102                PWMDir = false;
	CLR  R5
; 0000 0103                tampUpStop = true;
	LDI  R30,LOW(1)
	MOV  R9,R30
; 0000 0104                PORTB &= ~PB3;
	CBI  0x18,3
; 0000 0105                ledArrowControl('u', false);
	LDI  R30,LOW(117)
	ST   -Y,R30
	RCALL SUBOPT_0x4
; 0000 0106                break;
	RJMP _0x29
; 0000 0107            case 3:
_0x2A:
	CPI  R30,LOW(0x3)
	LDI  R26,HIGH(0x3)
	CPC  R31,R26
	BRNE _0x2C
; 0000 0108                PORTB |= PB6;
	SBI  0x18,6
; 0000 0109                statusMotor = 0;
	CLR  R4
; 0000 010A                PWMDir = false;
	CLR  R5
; 0000 010B                tampDownStop = true;
	LDI  R30,LOW(1)
	MOV  R8,R30
; 0000 010C                PORTB &= ~PB3;
	CBI  0x18,3
; 0000 010D                ledArrowControl('d', false);
	LDI  R30,LOW(100)
	ST   -Y,R30
	RCALL SUBOPT_0x4
; 0000 010E                break;
; 0000 010F            default: break;
_0x2C:
; 0000 0110        }
_0x29:
; 0000 0111 
; 0000 0112     }
; 0000 0113     else return;
	RJMP _0x2D
_0x26:
	RJMP _0x3A
; 0000 0114 
; 0000 0115 }
_0x2D:
	RJMP _0x3A
; .FEND
;
;interrupt [EXT_INT1] void exterInt1(void)     // interupt buttom control
; 0000 0118 {
_exterInt1:
; .FSTART _exterInt1
	RCALL SUBOPT_0x5
; 0000 0119 
; 0000 011A         if(!statusMotor) {numButtom = checkPortPC(0, 1);}
	BRNE _0x2E
	LDI  R30,LOW(0)
	LDI  R31,HIGH(0)
	ST   -Y,R31
	ST   -Y,R30
	LDI  R26,LOW(1)
	LDI  R27,0
	RCALL _checkPortPC
	MOVW R6,R30
; 0000 011B 
; 0000 011C         switch(numButtom)
_0x2E:
	MOVW R30,R6
; 0000 011D         {
; 0000 011E             case 0:
	SBIW R30,0
	BRNE _0x32
; 0000 011F 
; 0000 0120                 if(tampUpStop) return;
	TST  R9
	BRNE _0x3A
; 0000 0121                 else
; 0000 0122                 {
; 0000 0123                     PORTD |= PD7;
	SBI  0x12,7
; 0000 0124                     tampDownStop = false;
	CLR  R8
; 0000 0125                     PORTB &= ~PB6;
	CBI  0x18,6
; 0000 0126                     goUpDown(1);
	LDI  R26,LOW(1)
	RCALL _goUpDown
; 0000 0127                     PORTD &= ~PD7;
	CBI  0x12,7
; 0000 0128                 }
; 0000 0129 
; 0000 012A                 break;
	RJMP _0x31
; 0000 012B             case 1:
_0x32:
	CPI  R30,LOW(0x1)
	LDI  R26,HIGH(0x1)
	CPC  R31,R26
	BRNE _0x38
; 0000 012C 
; 0000 012D                 if(tampDownStop) return;
	TST  R8
	BRNE _0x3A
; 0000 012E                 else
; 0000 012F                 {
; 0000 0130                     tampUpStop = false;
	CLR  R9
; 0000 0131                     PORTB &= ~PB5;
	CBI  0x18,5
; 0000 0132                     goUpDown(0);
	LDI  R26,LOW(0)
	RCALL _goUpDown
; 0000 0133                 }
; 0000 0134                 break;
	RJMP _0x31
; 0000 0135 
; 0000 0136             default:{PORTD |= PD6;PORTD &= ~PD6;break;}
_0x38:
	SBI  0x12,6
	CBI  0x12,6
; 0000 0137         }
_0x31:
; 0000 0138 
; 0000 0139 }
_0x3A:
	LD   R30,Y+
	OUT  SREG,R30
	LD   R31,Y+
	LD   R30,Y+
	LD   R27,Y+
	LD   R26,Y+
	LD   R25,Y+
	LD   R24,Y+
	LD   R23,Y+
	LD   R22,Y+
	LD   R15,Y+
	LD   R1,Y+
	LD   R0,Y+
	RETI
; .FEND

	.CSEG
;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:5 WORDS
SUBOPT_0x0:
	LDI  R30,LOW(0)
	OUT  0x22,R30
	OUT  0x25,R30
	OUT  0x24,R30
	OUT  0x23,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x1:
	IN   R1,18
	LDD  R30,Y+6
	LDI  R26,LOW(1)
	RCALL __LSLB12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:2 WORDS
SUBOPT_0x2:
	OUT  0x23,R30
	LDI  R30,LOW(108)
	OUT  0x25,R30
	LD   R30,Y
	ST   -Y,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x3:
	OUT  0x23,R30
	LDI  R26,LOW(7)
	LDI  R27,0
	RJMP _delay_ms

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x4:
	LDI  R26,LOW(0)
	RJMP _ledArrowControl

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:11 WORDS
SUBOPT_0x5:
	ST   -Y,R0
	ST   -Y,R1
	ST   -Y,R15
	ST   -Y,R22
	ST   -Y,R23
	ST   -Y,R24
	ST   -Y,R25
	ST   -Y,R26
	ST   -Y,R27
	ST   -Y,R30
	ST   -Y,R31
	IN   R30,SREG
	ST   -Y,R30
	TST  R4
	RET


	.CSEG
_delay_ms:
	adiw r26,0
	breq __delay_ms1
__delay_ms0:
	__DELAY_USW 0xFA
	wdr
	sbiw r26,1
	brne __delay_ms0
__delay_ms1:
	ret

__LSLB12:
	TST  R30
	MOV  R0,R30
	MOV  R30,R26
	BREQ __LSLB12R
__LSLB12L:
	LSL  R30
	DEC  R0
	BRNE __LSLB12L
__LSLB12R:
	RET

__LSLW12:
	TST  R30
	MOV  R0,R30
	MOVW R30,R26
	BREQ __LSLW12R
__LSLW12L:
	LSL  R30
	ROL  R31
	DEC  R0
	BRNE __LSLW12L
__LSLW12R:
	RET

__SAVELOCR4:
	ST   -Y,R19
__SAVELOCR3:
	ST   -Y,R18
__SAVELOCR2:
	ST   -Y,R17
	ST   -Y,R16
	RET

__LOADLOCR4:
	LDD  R19,Y+3
__LOADLOCR3:
	LDD  R18,Y+2
__LOADLOCR2:
	LDD  R17,Y+1
	LD   R16,Y
	RET

;END OF CODE MARKER
__END_OF_CODE:
