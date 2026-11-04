import copy
import pathlib
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from cad_import import audit


class ImportTests(unittest.TestCase):
    def setUp(self):
        self.path = pathlib.Path(__file__).resolve().parents[1] / 'mechanical/import_fixture/base_link.urdf'
        self.root = ET.parse(self.path).getroot()

    def check(self):
        return audit(self.root, self.path.parent)

    def test_single_axis_si_properties_are_reported(self):
        report = self.check()
        self.assertEqual(report['joint']['axis'], [0., -1., 0.])
        self.assertEqual(report['links']['link']['mass_kg'], .1)
        self.assertEqual(report['links']['link']['center_of_mass_m'], [.075, 0., 0.])

    def test_non_finite_or_missing_mass_is_rejected(self):
        self.root.find('link/inertial')  # Base may be fixed without inertia.
        mass = self.root.find("link[@name='link']/inertial/mass")
        for value in ('nan', 'inf', '0', '-1'):
            mass.set('value', value)
            with self.assertRaises(ValueError): self.check()

    def test_non_physical_inertia_is_rejected(self):
        inertia = self.root.find("link[@name='link']/inertial/inertia")
        for changes in ({'ixx':'-1'}, {'ixy':'1'}, {'ixx':'1'}):
            saved = copy.deepcopy(inertia.attrib)
            inertia.attrib.update(changes)
            with self.assertRaises(ValueError): self.check()
            inertia.attrib.clear(); inertia.attrib.update(saved)

    def test_multiple_dof_and_zero_axis_are_rejected(self):
        self.root.find('joint/axis').set('xyz', '0 0 0')
        with self.assertRaises(ValueError): self.check()
        self.root.find('joint/axis').set('xyz', '0 -1 0')
        self.root.append(copy.deepcopy(self.root.find('joint')))
        with self.assertRaises(ValueError): self.check()

    def test_unresolved_mesh_and_wrong_length_scale_are_rejected(self):
        geometry = self.root.find("link[@name='link']/visual/geometry")
        geometry.clear(); ET.SubElement(geometry, 'mesh', filename='missing.stl')
        with self.assertRaises(ValueError): self.check()
        geometry.clear(); ET.SubElement(geometry, 'box', size='150 20 20')
        with self.assertRaises(ValueError): self.check()

    def test_unknown_parent_or_disconnected_link_is_rejected(self):
        self.root.find('joint/parent').set('link', 'missing')
        with self.assertRaises(ValueError): self.check()
        self.root.find('joint/parent').set('link', 'base')
        ET.SubElement(self.root, 'link', name='orphan')
        with self.assertRaises(ValueError): self.check()


if __name__ == '__main__': unittest.main()
