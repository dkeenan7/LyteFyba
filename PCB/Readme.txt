In February 2012, the PCB design documents for the Battery Management Units (BMUs)
changed from Protel format to DesignSpark format. DesignSpark runs on Windows and
is free, and can be downloaded from http://www.designspark.com/knowledge/pcb .
DesignSpark may run in Linux under Wine, with some difficulty (mainly the installer,
it seems). Read the latest forum comments in this if you want to use Linux.
Activation is required, free, and fairly low pain; just install and run the
software, and follow the prompts.

We want to keep the CMU-specific library files with the other design documents, so
it's unfortunately necessary to do the following if you want to make detailed
changes to the circuit or PCB layout. (It is NOT needed for perusing the circuit
or layout, or even making minor changes.)

Double-click the "CellTopBMU.sch" file to open it in DesignSpark; it's in the PCB
folder. Close the little window that contains nothing but the DesignSpark logo.
Open DesignSpark's Library Manager by clicking the book-like icon on the toolbar
(or typing Ctrl+L). Choose the "Folders" tab (important). Click on the "Add" button,
then the "Browse" button. You should see four "Coulomb&Weber" files. If not, navigate
to the celltopcmu/PCB folder so you do see them. Double-click any one of them (or
type Backspace Enter). Then click "OK" to complete the Add operation.

Now in the lower "Files Found" list you should see the same three "Coulomb&Weber"
files. If any are greyed out, select them and check the "Enabled" checkbox.

To check that you've done it right, choose the "Components" tab (in the Libraries dialog
box, Ctrl+L if not open), and in the "Libraries" drop down list, you should see near the
top "Coulomb&Weber.cml in ..."; select it. Now you should see a few dozen components,
including "Dual diode", which we did not find in the standard DesignSpark libraries.
Click "Close".

Rotatables Grouping Procedure
=============================
The section below is for creating the named group "Rotatables". It's a lot of work, so
don't do it unless you need to; just use the group already created. It can be accessed
in the interaction bar (F9 to display, if not visible). Choose the Goto tab at the bottom,
then the Group selection in the dropdown box near the top of the interaction bar. Double
clicking on the Rotatable or other named group should highlight the elements of that
group. If not, fiddle with the right button menu items like "Select all find items" until
it does.

To recreate the Rotatables group:

* Delete any previous Rotatables group. Do this by double clicking on the old group name
to highlight the elements, right click on one, and choose Group / Ungroup. The group name
should disappear from the interaction bar.
* Show all layers except Top and Bottom Documentation.
* Ensure only the copper-pour-area outlines are displayed, not their fills.
Context menu in space and choose Clear Copper if necessary.
* Make a rectangular selection from -2500,300 to 2500,-300. The 2500 numbers are
not critical, as long as everything is selected in the middle section. However the 300
numbers are critical. Using the arrow keys allows better accuracy and will (mostly) snap to
grid.
* Add the thick red GND track north of the processor, and the thin red Vdd track
south of R7, if necessary.
* Add R9 and its tracks and vias to the selection.
* Add all bypass resistors to the selection (easier with one layer displayed at a time).
* Add the snap-off holes for the bypass outriggers.
* Add all copper pour outlines associated with bypass resistors to the selection (not the copper 
interior, just the outlines). Ctrl-Shift-Clicking is useful here.
* Remove the board outline, including the oval holes, from the selection. Ctrl-Shift-Clicking is useful here.
* From the top and bottom silkscreens, deselect all text except component designators.
* Deselect the arrows (objects can only be a member of one group). Leave the plus and minus signs selected.
* Deselect everything from the top and bottom documentation layers, if necessary.
* Using the layers tab in the interaction bar, check each layer for items to deselect.
* Select any tracks that connect selected objects. Ctrl-Shift-Clicking is useful here.
* Context menu select "Group", uncheck "Tight" and give the group a name such as "Rotatables"

Rotation Procedure
==================
Click in space to deselect all
Context menu in space and choose Clear Copper
Context menu on the left cell-terminal pad and choose Group / Select Group
Press R twice
Press shift-"+" and enter 2440 into the X field and OK.
If the above doesn't work because it no longer rotates about the left terminal pad,
manually overlap (just barely) the blue blobs where the rotatables connect to the non-rotatables.
Click in space to deselect all
Context menu on the arrow near the left cell-terminal pad and choose Group / Select Group.
This assumes the existence of an "Arrows" group containing both arrows.
Press F
Press shift-"+" and enter 1780 into the X field and OK
Click in space to deselect all
Context menu in space and choose Pour Copper


Panelisation Procedure
======================
Copy or Save As the .pcb file to a suitable name
Zoom out; leave room below; 
Ensure all layers are visible
Frame select the whole BMU including the squiggle-join cutouts above
Deselect board outline including the oval holes, by Ctrl-Shift-click.
Copy and Paste.
When the Paste Net dialog comes up, select All No
Press shift-"+" and enter 0 into the X field and -1810 into the Y field. OK
Click in space to deselect all
Delete the mounting hole and its keepout circle (a tight group) from the original.
Apply the rotation procedure to the north (original) board only
Now is a good time for a design rule check.
Make all layers visible
Zoom out; select the two BMUs; deselect outline; copy paste; All No
Press shift-"+" and enter X = 0 and Y = 3620
Zoom out; select all 4 BMUs; deselect outline; copy; paste; All No
Press shift-"+" and enter X = 0 and Y = 7240
Zoom in near the top of the 8 BMUs
Select around the top squiggle join; delete
Remove dangling tracks and via at top and dangling tracks at bottom.
Deselect all; Context menu in space Pour copper (may not be necessary)
Save. Output manufacturing plots: Output / Manufacturing Plots; Select at least one
plot (e.g. Cutouts Text), use Layers tab to turn on "Board outline" for this plot.

