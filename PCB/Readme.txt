In February 2012, the PCB design documents for the Battery Management Units (BMUs)
changed from Protel format to DesignSpark format. DesignSpark runs on Windows and
is free, and can be downloaded from http://www.designspark.com/knowledge/pcb .
Activation is required, free, and low pain.

We want to keep the BMU-specific library files with the other design documents, so
it's unfortunately necessary to do the following if you want to make detailed
changes to the circuit or PCB layout. (It is NOT needed for perusing the circuit
or layout, or even making minor changes.)

Start the DesignSpark PCB application, either from the start menu or by double
clicking on the CellTopBMU.sch file. Open the Library Manager with control-L or by
clicking the book (fifth) toolbar button. Choose the Folders (last) tab. Click on
the Add button, then either type the path to the celltopBMU\PCB folder, or use the
Browse button to locate it. Click OK, and Close. That's it.

To check that you've done it right, open the library managed again, choose the
Components (middle) tab, and in the Libraries drop down list, you should see near
the top "Coulomb&Weber.cml in ..."; select it. Now you should see a handful of
components, including "Dual diode", which we did not find in the standard
DesignSpark libraries.