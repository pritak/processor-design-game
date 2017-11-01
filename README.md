# Processor Design Game: Tic-Tac-Toe
The final report is included in the list of files. Take a look at that for more information about the project, deliverables, and in depth analysis of our code.

## Files
### FinalProjectReport.pdf
The final project report for the entire group detailing the design, challenges, circuits, and code logic used in the final working project. Contains pictures and diagrams of the the working project to better explain what was done. 

### ProcessorUserForGame.v
This verilog file contains the actual processor that was used in the demo before final features such as bex and setx were incorporated. This file uses the blt instruction like beq, which was the logic needed for the code we wrote.

### beqInstruction.v
This verilog file contains one of the final versions of our processor with working mult/div and bex and setx implemented. This file uses the blt instruction like beq, which was the logic needed for the code we wrote. 

### bltInstruction.v
This verilog file contains one of the final versions of our processor with working mult/div and bex and setx implemented. This file uses the normal blt instruction. 

### tictactoeCode.asm
This asm file contains the assembly code used to implement our tic-tac-toe game logic.

### dmem.mif and imem.mif
The compiled memory initialization files for the tictactoeCode obtained using the assembulator.

## Collaborators
1. Pritak Patel, Duke University
2. Anika Radiya-Dixit, Duke University
3. Bruna Liborio, Duke University
