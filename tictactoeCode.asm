.text
main:
add $r0, $r0, $r0
addi $r21, $r0, 77
addi $r1, $r0,1
addi $r2, $r0,2
addi $r17, $r0,9
addi $r6, $r0, 21
addi $r7, $r0, 29
addi $r8, $r0, 36
addi $r9, $r0, 28
addi $r10, $r0, 27
addi $r11, $r0, 35
addi $r12, $r0, 26
addi $r13, $r0, 34
addi $r14, $r0, 33
addi $r16, $r0, 0
bne $r3, $r1, setturn1
bne $r3, $r2, setturn2
reset:
add $r0, $r0, $r0
addi $r21, $r0, 77
blt $r21, $r1, reset
addi $r1, $r0,1
addi $r2, $r0,2
addi $r17, $r0,9
addi $r6, $r0, 21
addi $r7, $r0, 29
addi $r8, $r0, 36
addi $r9, $r0, 28
addi $r10, $r0, 27
addi $r11, $r0, 35
addi $r12, $r0, 26
addi $r13, $r0, 34
addi $r14, $r0, 33
addi $r16, $r0, 0
bne $r3, $r1, setturn1
bne $r3, $r2, setturn2
setturn2:
add $r0, $r0, $r0
addi $r3, $r2, 0
j player2turn
setturn1:
add $r0, $r0, $r0
addi $r3, $r1, 0
j player1turn
player1turn:
add $r0, $r0, $r0
blt $r15, $r6, qslot
blt $r15, $r7, wslot
blt $r15, $r8, eslot
blt $r15, $r9, aslot
blt $r15, $r10, sslot
blt $r15, $r11, dslot
blt $r15, $r12, zslot
blt $r15, $r13, xslot
blt $r15, $r14, cslot
j player1turn
player2turn:
add $r0, $r0, $r0
blt $r15, $r6, qslot
blt $r15, $r7, wslot
blt $r15, $r8, eslot
blt $r15, $r9, aslot
blt $r15, $r10, sslot
blt $r15, $r11, dslot
blt $r15, $r12, zslot
blt $r15, $r13, xslot
blt $r15, $r14, cslot
j player2turn
qslot:
add $r0, $r0, $r0
addi $r6, $r3, 0
j checkback
wslot:
add $r0, $r0, $r0
addi $r7, $r3, 0
j checkback
eslot:
add $r0, $r0, $r0
addi $r8, $r3, 0
j checkback
aslot:
add $r0, $r0, $r0
addi $r9, $r3, 0
j checkback
sslot:
add $r0, $r0, $r0
addi $r10, $r3, 0
j checkback
dslot:
add $r0, $r0, $r0
addi $r11, $r3, 0
j checkback
zslot:
add $r0, $r0, $r0
addi $r12, $r3, 0
j checkback
xslot:
add $r0, $r0, $r0
addi $r13, $r3, 0
j checkback
cslot:
add $r0, $r0, $r0
addi $r14, $r3, 0
j checkback
checkback:
add $r0, $r0, $r0
checkvictory:
add $r0, $r0, $r0
blt $r6, $r7, checkrow1
row2check:
add $r0, $r0, $r0
blt $r9, $r10, checkrow2
row3check:
add $r0, $r0, $r0
blt $r12, $r13, checkrow3
col1check:
add $r0, $r0, $r0
blt $r6, $r9, checkcol1
col2check:
add $r0, $r0, $r0
blt $r7, $r10, checkcol2
col3check:
add $r0, $r0, $r0
blt $r8, $r11, checkcol3
dia1check:
add $r0, $r0, $r0
blt $r6, $r10, checkdia1
dia2check:
add $r0, $r0, $r0
blt $r8, $r10, checkdia2
j changeturn
checkrow1:
add $r0, $r0, $r0
blt $r6, $r8, victory
j row2check
checkrow2:
add $r0, $r0, $r0
blt $r9, $r11, victory
j row3check
checkrow3:
add $r0, $r0, $r0
blt $r12, $r14, victory
j col1check
checkcol1:
add $r0, $r0, $r0
blt $r6, $r12, victory
j col2check
checkcol2:
add $r0, $r0, $r0
blt $r7, $r13, victory
j col3check
checkcol3:
add $r0, $r0, $r0
blt $r8, $r14, victory
j dia1check
checkdia1:
add $r0, $r0, $r0
blt $r6, $r14, victory
j dia2check
checkdia2:
add $r0, $r0, $r0
blt $r8, $r12, victory
changeturn:
add $r0, $r0, $r0
addi $r16, $r16, 1
blt $r16, $r17, winlogic
toturn2:
add $r0, $r0, $r0
bne $r3, $r1, toturn1
addi $r3, $r2, 0
j player2turn
toturn1:
add $r0, $r0, $r0
bne $r3, $r2, toturn2
addi $r3, $r1, 0
j player1turn
victory:
add $r0, $r0, $r0
blt $r3, $r1, player1wins
blt $r3, $r2, player2wins
player1wins:
add $r0, $r0, $r0
addi $r4, $r4, 1
j winlogic
player2wins:
add $r0, $r0, $r0
addi $r5, $r5, 1
j winlogic
winlogic:
add $r0, $r0, $r0
blt $r15, $r21, reset
j winlogic
add $r0, $r0, $r0
add $r0, $r0, $r0
add $r0, $r0, $r0
add $r0, $r0, $r0
.data
