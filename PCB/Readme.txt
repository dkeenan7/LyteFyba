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

Double-click the "MidCMU.sch" file to open it in DesignSpark; it's in the PCB
folder. Close the little window that contains nothing but the DesignSpark logo.
Open DesignSpark's Library Manager by clicking the book-like icon on the toolbar
(or typing Ctrl+L). Choose the "Folders" tab (important). Click on the "Add" button,
then the "Browse" button. You should see 3 or 4 "Coulomb&Weber" files. If not, navigate
to the LyteFyba/trunk/PCB folder so you do see them. Double-click any one of them (or
type Backspace Enter). Then click "OK" to complete the Add operation.

Now in the lower "Files Found" list you should see the same 3 or 4 "Coulomb&Weber"
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

* Ungroup any previous Rotatables group. Do this by double clicking on the old group name
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
* Remove the board outline from the selection but keep the terminal holes. Ctrl-Shift-Clicking is
useful here.
* From the top silkscreen, deselect any horizontal text except component designators.
* Leave the plus and minus signs selected.
* Deselect everything from the top and bottom documentation layers, and Cutouts text, if necessary.
* Using the layers tab in the interaction bar, check each layer for items to deselect.
* Select any tracks that connect selected objects. Ctrl-Shift-Clicking is useful here.
* Select the mouse-bite holes on the wings.
* Context menu on one of the selected objects and select "Group", uncheck "Tight Group" 
and give the group the name "Rotatables"

Rotation Procedure
==================
Ensure all layers are visible, except the two documentation layers and Cutouts text.
Click in space to deselect all. Context menu in space and choose Clear Copper if it's available.
Select SG1, e.g. by clicking on one of its pads. 
Context menu on SG1 and choose Group / Select Group. Press R twice.
If the above doesn't work because it no longer rotates about the origin, then
manually almost-overlap the blue blobs where the rotatables connect to the non-rotatables, then
remember the new dx and dy figures from the bottom of the window to use in a + command from
now on. Click in space to deselect all.
Context menu in space and choose Pour Copper (with Remove isolated islands). Design check. 
Check for silkscreen on pads and fix any. Ignore it if "Fyba" is on pads. That will be fixed later.


Replication Procedure (Panel of 12 CMUs for prismatic cells)
================================================================
Copy or Save As the .pcb file to a suitable name
Display only bottom silkscreen and bottom copper layers. 
Unroute (^U) the blue track segment immediately to the to the left of the south pad of R1.
Select R1 and delete it. Add a new track starting from the end of the printed resistor
and going up to the now unconnected via. Ok the warning that this will create one net Vdd from Vdd
and BypR; this is unavoidable. Design rule check should still pass. 
If necessary, add the *two* new track segments (one is small under the via) to the Rotatables group
(select them, use context menu "Add to group", choose the "Rotatables" group, uncheck "Tight Group").
If "Add to group" is not available they may have been automatically added to the group.
Check this when you perform the rotation later.
Display top copper as well.
Ensure the 6 or 7 copper pours have their outline width set to the minimum track width e.g. 0.2 mm. 
Pour all copper (with Remove isolated islands). Design check. Clear all copper pours.
Zoom out.
Apply the above rotation procedure.
Ensure all layers are visible, except the two documentation layers and Cutouts text layer.
Frame select the whole CMU.
Use Ctrl-Shift-clicks to deselect outlines of CMUs, fill-ins, tabs and tooling hole. 
Keep the cell-terminal cutouts selected. Zoom out, leaving room to the left.
Copy and Paste.
When the Paste Net dialog comes up, select All No
Press shift-"+" and enter -85.5 into the X field and 21 into the Y field (-83.5 and 21 for the MidCMU). OK.
Click in space to deselect all.
Apply the rotation procedure again, to the original CMU only, to return it to normal.
Change "CMU-A" to "CMU-B" in the copy.
You may need to move the text "Fyba" to the diagonally opposite position in the copy.
Now is a good time for a design rule check. We expect zero errors.
Also check that no parts on the copy have had their designators changed. 
They should be displaying Name values rather than component names.
Now is also a good time to generate the pick and place file.
Make all layers visible except the two Doc layers and Cutouts text. Clear copper pours. Zoom out. 
Select the two CMUs. Deselect board outline, tabs, tooling holes and fiducial. 
Copy and paste. All No. Press shift-"+" and enter X = 0 and Y = 42.8 (0 and 42 for the MidCMU). OK.
Repeat the above 4 more times, substituting the following values for Y. 85.6, 128.4, 171.2, 214 (84, 126, 168, 210 for the MidCMU).
Deselect all. Pour copper.
Save. Follow the procedure below to generate the manufacturing files.

Isomerisation Procedure (Octagonal CMUs for cylindrical cells)
==============================================================
Save As the .pcb file twice, to two suitable names e.g. CMUp and CMUn.

From CMUn:
Delete the large minus sign north of F2.
Delete the Strap- pad north of F2, and its associated strain relief hole.
Delete fuse F2 and the "F2" text.
Turn the middle plus sign into a minus sign by deleting the vertical stroke.
Delete the dangling track going northeast from the large central terminal pad.

From CMUp:
Delete the large plus sign north of D1 (two lines).
Delete the Strap+ pad east of D1, and its associated strain relief hole.
Delete fuse F1 and the "F1" text. This will also delete a segment of VDD�track to its southwest.
Reconnect from the VDD track to the via at the north end of R1, and from this via to the south end
of D1, using Power Min track style.
Delete the large GND track going west from the large central terminal pad.
Change the net of the large central terminal pad (check the "Change Name of Subnet Only" checkbox)
from GND to Strap+. OK the warning, provided it says it will only change a subnet.
Complete the dangling Strap+ track going northeast from the central terminal pad, by joining it 
to the Strap+�track going to D1, using Power Nom style.

On both CMUs:
Cut (Control-U, NOT delete) the blue track segment south of R1. Delete R1. Add a new track from the dangling
end of the printed resistor to the VDD via just north of where R1 was. OK the warning.
Delete the IFO devices Q2 and D7 north of the PCB.
Ensure that all copper pours (there's only one and it's in the top layer) have their outline width 
set to the minimum track width e.g. 0.2 mm.
Deselect all; Context menu in space and Pour copper.
Perform a Design Rule Check; there should be no errors.
Save. Follow the procedure below to generate the manufacturing files.

Generate manufacturing files
============================
Choose Output / Manufacturing Plots.
Select the Cutouts Text plot. Use Layers tab to turn on "Board outline" for this plot.
For OSH Park and PCB Zone use 3:3 metric for gerbers and 3:4 metric for drill files 
and ensure the outline is not included in any gerber or drill files
except the gerber generated from the Cutouts text layer.
To do this, check the Settings tab of the Drill Data plot and uncheck both Unplated Board outlines and 
Plated Board Outlines.
For OSH Park, generate a single (combined plated and non-plated) drill file. 
OSH Park use the presence of copper under the drill to distinguish plated from non-plated.
For PCB Zone, generate separate plated and non-plated drill files.
To change this see the Output tab of the Drill Data plot and use the Device Setup button.
Check that the two Paste gerbers contain only their appropriate Paste layer.

Don't send any bottom silkscreen gerber to the manufacturer. 
It only contains thru-hole part outlines that we didn't want in the top silkscreen.
And we don't need a Bottom paste mask either. It should be empty.

Reasons for using lowercase letters after component designator numbers
======================================================================
1. In same package (enforced by DesignSpark)
2. In series
3. In parallel
4. Same function and same value repeated in one PCB
5. BMU part with same function as in CMU but different value
6. (Obsolete) Alternative part locations (now indicated by e.g. R7 and R7alt)