.. _circuit_design:

Circuit Design
==============

The Schematics
--------------

.. raw:: html

   <iframe src="../_static/hardware/SCH_DOGlove_MainCircuit.pdf" width="100%" height="600px"></iframe>

Import the Project File
------------------------

We use lceda (EasyEDA) to design the schematic and PCB layout. To preview the source project, you have two options:

Option 1: Import the `.epro` File
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- **For users in China**: Use the `lceda Pro Edition (立创eda专业版) <https://pro.lceda.cn/editor>`__ to import the `.epro` file.
- **For users in other regions**: Use the `EasyEDA Pro Edition <https://pro.easyeda.com/editor>`__ to import the `.epro` file.

Steps to Import:

1. Download the :download:`ProPrj_DOGlove_MainBoard.epro <../_static/hardware/ProPrj_DOGlove_MainBoard.epro>` file.
2. Open EasyEDA Pro and navigate to ``File → Import → EasyEDA (Professional)``.

   .. image:: ../_static/hardware/1_import_project.jpg

3. Upload the `.epro` file, adjust the settings as shown below, then click **Import**:
   
   .. image:: ../_static/hardware/2_project_setting.jpg

4. Confirm the import settings and click **Save**:

   .. image:: ../_static/hardware/3_import_setting.jpg

5. You should now see the full schematic and layout preview:

   .. image:: ../_static/hardware/4_project_preview.jpg

Option 2: View on OSHWHub (Recommended for Users in China)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can directly view the `DOGlove project on OSHWHub (立创开源硬件平台) <https://oshwhub.com/doublehan/doglove_mainboard>`__, which includes the schematics, PCB layout, and BOM. The platform also supports one-click PCB fabrication.
