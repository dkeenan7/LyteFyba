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
To be advised

Rotation Procedure
==================
Context menu in space and select Clear Copper
Hover over the left terminal pad; context menu Group / Select Group
Press R twice
Press "+" and enter 2440 into the X field
Click in space; context menu Pour Copper


