This project now requires Visual Studio 2015 or later; Community Edition (free
download) is fine.

Installing VS 2015
==================
Note that this project uses Microsoft Foundation Classes (MFC), which are now
considered non-mainstream, so you have to explicitly select the MFC option
when installing. Choose a Custom Install and under the Programming Languages
/ Visual C++ selections (use the small buttons to the left of these option
names to view their sub-options) ensure that all three of these are selected:
* Common Tools...
* Microsoft Foundation Classes...
* Windows XP Support...
The latter is needed even if you don't care about Windows XP support, because
we've enabled this inside the project file (CMUsend.vcproj) and presumably it
won't compile without the XP support having been installed.
[ If you really don't want XP support for some reason or forget to install it,
you could remove the XP support option by editing the CMUsend.vcproj project
file. It's actually easier to edit it outside of Visual Studio. There are
three places where you'll need to change the string "V140_xp" (without quotes)
back to "V140" (without quotes). ]

You do NOT have to choose a Windows Software Development Kit (SDK). In other
words, you don't need anything selected under Windows and Web Development. If
you do choose an SDK, be prepared for an even larger and slower install.

Compiling CMUsend with VS 2015
==============================

To reproduce the distributed executable and to save disk space, choose the
Release solution configuration (Debug is default). The drop-down list box can
be found to the right of the toolbar imaged buttons (icons) on a standard
installation of Visual Studio.

Building the executable is standard; no fancy options are needed (other than
support for Windows XP; the project file has been modified to enable that
option. Note that it is not possible to choose XP support or not through any
toolbar, menu, or other GUI method.) To compile and link, use the Build /
Build Solution menu option.

Later VS Compilers
==================

When later compilers are available, they may or may not be able to compile
CMUsend. Microsoft seems to have changed its mind over both MFC support and
Windows XP support, so it's impossible to predict whether such support will
continue for later versions of Visual Studio. If VS 2015 is still available,
it will by far be the most convenient option. You most likely will not be
making extensive changes that would benefit from the features of the latest
version of Visual Studio.

The executable provided as part of the GitHub distribution will hopefully
continue to work on newer versions of Windows, so this would only be an
issue if the functionality of CMUsend had to bechanged.