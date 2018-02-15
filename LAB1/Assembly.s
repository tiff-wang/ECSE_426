	AREA text, CODE, READONLY
	EXPORT asm_math

asm_math
; inputs:
;   R0 : pointer to input array
;   R1 : pointer to output array
;   R2 : length of input array
; output:
;   [RMS_value, max_value, min_value, max_index, min_index]

; R0 input array pointer
; R1 output array pointer
; R2 input array length
; R3 max index
; R4 min index
; R5 loop counter
; ---
; S0 rms
; S1 max 
; S2 min
; S3 for looping inputs
		PUSH {R4, R5}			
		MOV R5, #0				; counter = 0
		MOV R3, #0				; max, min index = 0
		MOV R4, #0	
		VSUB.F32 S0, S0, S0		
		VLDR.32 S1, [R0]		; set min and max = first
		VLDR.32 S2, [R0]
loop	CMP R5, R2				
		BEQ exit
		VLDR.32 S3, [R0]		; current item
		VFMA.F32 S0, S3, S3		; 
		
		VCMP.F32 S3, S1			; compare current item to current min
		VMRS APSR_nzcv, FPSCR	; transfer FP status register to ARM APSR
		BGT ifc					; 
		
		VCMP.F32 S3, S2			; compare current item to current max
		VMRS APSR_nzcv, FPSCR	; 
		BLT elsec				;
		
		ADD R5, R5, #1			; ++i
		ADD R0, R0, #4
		B loop
ifc		MOV R3, R5				; max 
		VMOV.F32 S1, S3
		ADD R5, R5, #1			; ++i
		ADD R0, R0, #4
		B loop
elsec	MOV R4, R5				; min
		VMOV.F32 S2, S3
		ADD R5, R5, #1			; ++i
		ADD R0, R0, #4
		B loop
exit	VMOV S3, R2				; convert length int->float
		VCVT.F32.U32 S3, S3		
		VDIV.F32 S0, S0, S3		; divide sum of squares 
		VSQRT.F32 S0, S0		; square root and add back
		VSTR.32 S0, [R1]		; mrs
		VSTR.32 S1, [R1, #4]	; max value
		VSTR.32 S2, [R1, #8]	; min value
		
		VMOV S0, R3				; move max index to floating point register
		VCVT.F32.U32 S0, S0		; tell S0 to interpret data as integer
		VSTR.32 S0, [R1, #12]	; store max index in fourth cell of output
		VMOV S0, R4				; move min index to S0
		VCVT.F32.U32 S0, S0		; tell S0 to interpret data as integer
		VSTR.32 S0, [R1, #16]	; store min index in fifth cell of output
		
		POP {R4, R5}			; 
		BX LR
		
END