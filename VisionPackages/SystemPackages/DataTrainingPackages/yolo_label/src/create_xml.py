#!/usr/bin/env python
import os
from lxml import etree
import cv2
def write(dict_list, path):
    root = etree.Element("annotation")
    etree.SubElement(root, "folder").text = os.path.basename(os.path.dirname(path))
    etree.SubElement(root, "filename").text = os.path.basename(path)
    etree.SubElement(root, "path").text = path

    source = etree.SubElement(root, "source")
    etree.SubElement(source, "database").text = "Unknown"

    size = etree.SubElement(root, "size")
    etree.SubElement(size, "width").text = "1280"
    etree.SubElement(size, "height").text = "720"
    etree.SubElement(size, "depth").text = "3"

    etree.SubElement(root, "segmented").text = "0"

    for n, l in dict_list.iteritems():
        obj = etree.SubElement(root, "object")
        etree.SubElement(obj, "name").text = str(n)
        etree.SubElement(obj, "pose").text = "Unspecified"
        etree.SubElement(obj, "truncated").text = "0"
        etree.SubElement(obj, "difficult").text = "0"

        bndbox = etree.SubElement(obj, "bndbox")
        etree.SubElement(bndbox, "xmin").text = str(l[0])
        etree.SubElement(bndbox, "ymin").text = str(l[1])
        etree.SubElement(bndbox, "xmax").text = str(l[2])
        etree.SubElement(bndbox, "ymax").text = str(l[3])

    code = etree.tostring(root, pretty_print=True)

    filename = os.path.splitext(os.path.basename(path))[0]+'.xml'
    filepath = os.path.join(os.path.dirname(path), filename)
    with open(filepath, 'w') as the_file:
        the_file.write(code)





