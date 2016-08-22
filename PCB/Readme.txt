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
* Show all layers except Top and Bottom Documentation, and Cutouts text.
* Ensure only the copper-pour-area outlines are displayed, not their fills.
Context menu in space and choose Clear Copper if necessary.
* Make a rectangular selection from -65,7.5 to 60,-7.5. The 65 and 60 numbers are
not critical, as long as everything is selected in the middle section. However the 7.5
numbers are critical. Using the arrow keys allows better accuracy and will (mostly) snap to
grid.
* Add the thick red GND and Vdd tracks south of the processor, (use control-shift-click to add
connected sections of track).
* Add thee red sections of track near the crystal to the selection.
* Add the printed bypass resistor to the selection by Ctrl-ShiftClick (may need clicking twice).
* Remove the board outline from the selection but keep the oval holes. Ctrl-Shift-Clicking is
useful here.
* From the top silkscreen, deselect any horizontal text except component designators.
* Leave the plus and minus signs selected.
* Deselect everything from the top and bottom documentation layers, and Cutouts text, if necessary.
* Using the layers tab in the interaction bar, check each layer for items to deselect.
* Select any tracks that connect selected objects. Ctrl-Shift-Clicking is useful here.
* Context menu on one of the selected objects and select "Group", uncheck "Tight" 
and give the group the name "Rotatables"

Rotation Procedure
==================
Ensure all layers are visible, except the two documentation layers and Cutouts text.
Click in space to deselect all. Context menu in space and choose Clear Copper if possible.
Select SG1. Use a horizontal piece of silkscreen "track" away from other tracks, as this
will minimise interference from tracks and pour outlines that may otherwise be selected by
mistake. This component has co-ordinates 0,0 , and is therefore ideal for use as the centre of the
rotation. Context menu on SG1 and choose Group / Select Group. Press R twice.
If the above doesn't work because it no longer rotates about the origin, then
manually almost-overlap the blue blobs where the rotatables connect to the non-rotatables, then
remember the new dx and dy figures from the bottom of the window to use in a + command from
now on. Click in space to deselect all.
Context menu in space and choose Pour Copper. Design check.


Replication Procedure and Preparing for Manufacture (Squiggle-joined CMUs for prismatic cells)
===================================================
Copy or Save As the .pcb file to a suitable name
Select only bottom silkscreen and bottom copper layers. Unroute (^U) the blue track connecting R1
with the rest of the printed resistor. Select R1 and delete it. Add a new track from the end of the
printed resistor to the now unconnected via. Ok the warning that his will create one net from Vdd
and BypR; this is unavoidable. Design rule check should still pass. Add the *two* new tracks (one is
small under the via) the rotatables group (select them, use contect menu "add to group" and choose
the "Rotatables" group).
Ensure all copper pours have their outline width set to the minimum track width e.g. 0.2 mm. Repour
all copper. Design check.
Zoom out.
Apply the above rotation procedure.  Clear all copper pours.
Ensure all layers are visible, except the two documentation layers and Cutouts text.
Frame select the whole CMU including the squiggle-join cutouts above, but not below.
Deselect board outline by Ctrl-Shift-click. Deselect squiggle-join cutouts below if necessary. Keep the
oval cutouts and squiggle-join cutouts. Zoom out; leave room below.
Copy and Paste.
When the Paste Net dialog comes up, select All No
Press shift-"+" and enter 0 into the X field and -46 into the Y field. OK
Click in space to deselect all
Delete the locating hole (north west corner) and its keepout circle (a tight group) from the original.
Apply the rotation procedure again, to the original CMU only, to return it to normal.
Now is a good time for a design rule check.
Make all layers visible, except the two documentation layers and Cutouts text.
Zoom out; select the two CMUs; deselect outline; copy paste; All No
Press shift-"+" and enter X = 0 and Y = 92
Zoom in near the top of the 4 CMUs
Select around the top squiggle join; delete
Remove dangling tracks at top and bottom.
Deselect all; Context menu in space and Pour copper, if necessary.
Save. Output manufacturing plots: Output / Manufacturing Plots; Select at least one
plot (e.g. Cutouts Text), use Layers tab to turn on "Board outline" for this plot.

Isomerisation Procedure and Preparing for Manufacture (Octagonal CMUs for cylindrical cells)
===================================================
Copy or Save As the .pcb file twice, to two suitable names e.g. CMUp and CMUn.

From CMUn:
Delete fuse F2 and its associated text.
Delete the left-hand large minus sign.
Turn the middle plus sign into a minus sign by deleting the vertical stroke.
Delete the pad for Strap- and its associated strain relief hole.
Delete the dangling Strap+ track from the large terminal pad.

From CMUp:
Delete fuse F1 and its associated text.
Reconnect from the VDD track to the via and connect from D1 to the via, using Power Min track style.
Delete the right-hand large plus sign.
Delete the pad for Straq+ and its associated strain relief hole.
Delete the GNE track from the large terminal pad.
Change the net of the large terminal pad (subnet only) to Straq+.
Complete the dangling Straq+ track so it's no longer dangling.

On both CMUs:
Add a small piece of dangling track to the bypR track (the printed resistor track) 
close to where it connects to the 0R resistor.
Delete the 0R.
Drag the end of the dangling track (without first selecting anything) up to the centre 
of the VDD via that used to be connected to the other side of the 0R.
Don't worry about the fact that the whole printed resistor is now a dangling track. 
This will give rise to one T-V error and one dangling track error per CMU. Ignore these errors. 
In fact delete all DRC error notifications, so as not to confuse the manufacturer.

Ensure all copper pours have their outline width set to the minimum track width e.g. 0.2 mm.
Deselect all; Context menu in space and Pour copper.
Save. Output manufacturing plots: Output / Manufacturing Plots; Select at least one
plot (e.g. Cutouts Text), use Layers tab to turn on "Board outline" for this plot.
For OSH Park use 3:3 metric for gerbers and 3:4 metric for the single (combined plated and non-plated) drill file.
For OSH Park Ensure the drill file does not include the outline.

Reasons for using lowercase letters after component designator numbers
======================================================================
1. In same package (enforced by DesignSpark)
2. In series
3. In parallel
4. Same function and same value repeated in one PCB
5. BMU part with same function as in CMU but different value
6. (Obsolete) Alternative part locations (now indicated by e.g. R7 and R7alt)