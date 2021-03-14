
;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------

   .def setPsp
   .def getPsp
   .def push_R4_R11
   .def pop_R4_R11
   .def get_R0

;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const

;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

; Blocking function that returns only when SW1 is pressed

setPsp:

	MSR PSP,R0;
	MOV R0,#2;
	MSR CONTROL,R0;
	ISB; // indian school of business, if il do MBA il go here and then il be the bouuseee
	BX LR;

getPsp:
	MRS R0,PSP;
	BX LR;

push_R4_R11:
	  MOV R4,#4  ;
      MOV R5,#5  ;
      MOV R6,#6  ;
      MOV R7,#7  ;
      MOV R8,#8  ;
      MOV R9,#9  ;
      MOV R10,#10  ;
      MOV R11,#11  ;

      MRS R0,PSP  ;
      STMDB R0!,{R4-R11}  ;
      MSR PSP,R0 ;
      BX LR;

pop_R4_R11:
	MRS R0,PSP;
	LDMIA R0!,{R4-R11};
	MSR PSP,R0;
	BX LR;

get_R0:
	BX LR;

.endm
