In February 2012, the PCB design documents for the Battery Management Units (BMUs)
changed from Protel format to DesignSpark format. DesignSpark runs on Windows and
is free, and can be downloaded from http://www.designspark.com/knowledge/pcb .
Activation is required, free, and low pain.

We want to keep the BMU-specific library files with the other design documents, so
it's unfortunately necessary to do the following if you want to make detailed
changes to the circuit or PCB layout. (It is NOT needed for perusing the circuit
or layout, or even making minor changes.)

Double-click the "CellTopBMU.sch" file to open it in DesignSpark; it's in the PCB
folder. Close the little window that contains nothing but the DesignSpark logo.
Open DesignSpark's Library Manager by clicking the book-like icon on the toolbar
(or typing Ctrl+L). Choose the "Folders" tab. Click on the "Add" button, then the
"Browse" button. You should see three "Coulomb&Weber" files. If not, navigate to
the celltopbmu/PCB folder so you do see them. Double-click any one of them (or
type Backspace Enter). Then click "OK" to complete the Add operation.

Now in the lower "Files Found" list you should see the same three "Coulomb&Weber"
files. If any are greyed out, select them and check the "Enabled" checkbox.

To check that you've done it right, choose the "Components" tab, and in the
"Libraries" drop down list, you should see near the top "Coulomb&Weber.cml in
..."; select it. Now you should see a handful of components, including "Dual
diode", which we did not find in the standard DesignSpark libraries. Click "Close".

Group Selection Procedure
=========================
Make a rectangular selection from -1600,300 to 1600,-300. Using the arrow keys allows
better accuracy and will (mostly) snap to grid
Add the large red GND track north of the processor, and if necessary a blue Vdd track
south of R7
Remove C6 from the selection
Add 6 bypass resistors to the selection (easier with red layers off)
Add 6 copper pour outlines to the selection (not the copper interior, just the outlines)
Remove the outline from the selection
From the bottom silkscreen, remove "Weber &", "Coulomb", "Digital", "BMU"
Remove everything from the top documentation layer
Using the layers tab in the interaction bar, check each layer for items to remove
Context menu select "Group" and give the group a name such as "Rotatables"

Rotation Procedure
==================
Context menu in space and select Clear Copper
Hover over the left terminal pad; context menu Group / Select Group
Press R twice
Press shift-"+" and enter 2440 into the X field
Click in space; context menu Pour Copper
Invert the arrows near the terminal pads

Panelisation Procedure
======================
Copy or Save As the .pcb file to a suitable name
Zoom out; leave room below; ensure all layers are visible; frame select the whole board
Deselect board outline; copy; paste
When the Paste Net dialog comes up, select All No
Drag the selected board south until the dx coordinate near the right end of the
status bar reads zero, and the dy coordinate reads -1810
Using the Z key (zoom) may help with mouse creep
Avoid "mouse creep"; click to place second board
Click in space to deselect all
Apply the rotation procedure to the north (original) board only
Zoom out; select the two boards; deselect outline; copy paste; All No
Drag the selected two boards north until dx = 0 and dy = 3620
Avoid creep; click to place next 2 boards
Zoom out; select all 4 boards; deselect outline; copy; paste; All No
Drag the selected 4 boards north until dx = 0 and dy = 7240
Avoid creep; click to place next 4 boards
Zoom in near the top of the 8 boards
Select around the top squiggle join; delete
Remove dangling tracks and via; repour all copper

