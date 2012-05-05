In February 2012, the PCB design documents for the Battery Management Units (BMUs)
changed from Protel format to DesignSpark format. DesignSpark runs on Windows and
is free, and can be downloaded from http://www.designspark.com/knowledge/pcb .
Versions of DesignSpark earlier than 3.0 can be run in Linux under Wine.
Activation is required, free, and fairly low pain; just install and run the
software, and follow the prompts.

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

Rotatables Grouping Procedure
=============================
Ungroup any previous Rotatables group.
Show all layers except Top Documentation, and ensure copper-pour-area outlines are displayed.
Make a rectangular selection from -1600,300 to 1600,-300. Using the arrow keys allows
better accuracy and will (mostly) snap to grid
Add the thick red GND track north of the processor, and the thin red Vdd track
south of R7, if necessary.
Remove C6 from the selection, if necessary.
Add 6 bypass resistors to the selection (easier with red layers off)
Add 6 copper pour outlines to the selection (not the copper interior, just the outlines)
Remove the board outline from the selection
From the bottom silkscreen, deselect "Weber &", "Coulomb", "Digital", "BMU"
Deselect everything from the top documentation layer, if necessary.
Using the layers tab in the interaction bar, check each layer for items to deselect
Context menu select "Group", uncheck "Tight" and give the group a name such as "Rotatables"

Rotation Procedure
==================
Click in space to deselect all
Context menu in space and choose Clear Copper
Context menu on the left cell-terminal pad and choose Group / Select Group
Press R twice
Press shift-"+" and enter 2440 into the X field and OK
Click in space to deselect all
Context menu in space and choose Pour Copper
Context menu on the arrow near the left cell-terminal pad and choose Group / Select Group
Press F
Press shift-"+" and enter 1780 into the X field and OK


Panelisation Procedure
======================
Copy or Save As the .pcb file to a suitable name
Zoom out; leave room below; 
Ensure all layers are visible
Frame select the whole BMU including the squiggle-join cutouts above
Deselect board outline by Ctrl-Shift-click; copy; paste
When the Paste Net dialog comes up, select All No
Press shift-"+" and enter 0 into the X field and -1810 into the Y field. OK
Click in space to deselect all
Delete the IFO daughter-board tab from the copy.
Apply the rotation procedure to the north (original) board only
Make all layers visible
Zoom out; select the two BMUs; deselect outline; copy paste; All No
Press shift-"+" and enter X = 0 and Y = 3620
Zoom out; select all 4 BMUs; deselect outline; copy; paste; All No
Press shift-"+" and enter X = 0 and Y = 7240
Zoom in near the top of the 8 BMUs
Select around the top squiggle join; delete
Remove dangling tracks and via at top and dangling tracks at bottom.
Deselect all; Context menu in space Pour copper

