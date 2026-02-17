.data					

.align  2				

.text		

median:
lw s1,0x8(x0) #s1 = ukazatel na zacatek pole
lw s2,0x4(x0) #s2 = pocet prvku
addi s3,x0,1 # 1 protoze posledni prvek se nemeni
addi t1,x0,1
addi t4,x0,4
for:
  beq  s3, s2, done
  beq s3,t1,rovna
  #nerovna
	  
	  # s5 vlevo, s6 uprostred, s7 vpravo
	  sub s1,s1,t4
	  lw s5,0(s1)
	  addi s1,s1,4
	  lw s6,0(s1)
	  lw s7,4(s1)
	  # s5 nejmensi, s6 uprostred, s7 nejvetsi, t2 temp
	  blt s5, s6, next1
	  add t2, s5, x0
	  add s5,s6,x0
	  add s6,t2,x0
	  next1:
	  blt s5, s7, next2
	  add t2, s5, x0
	  add s5,s7,x0
	  add s7,t2,x0
	  #v s5 je uz nejmensi
	  next2:
	  blt s6, s7, next3
	  add t2, s6, x0
	  add s6,s7,x0
	  add s7,t2,x0
	  next3:
  	  
  sw s6,0(s1)
  
  addi s1, s1, 4    # increment offset and move to the other value in the array
  addi s3, s3, 1    # increment number of passes through the cycle (i++).
  jal  x0,  for 
  
  rovna:
  addi s1, s1, 4
  addi s3, s3, 1 
  jal  x0,  for 
  
  
done:
jalr x0,ra,0