/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Popover view controller for choosing virtual objects to place in the AR scene.
*/

import Cocoa
import SceneKit

// MARK: - VirtualObjectSelectionViewControllerDelegate

/// A protocol for reporting which objects have been selected.
protocol VirtualObjectSelectionViewControllerDelegate: class {
    func virtualObjectSelectionViewController(_ selectionViewController: MacVirtualObjectSelection, didSelectObject: VirtualObject)
}

/// A custom table view controller to allow users to select `VirtualObject`s for placement in the scene.
public class MacVirtualObjectSelection {
    
    /// The collection of `VirtualObject`s to select from.
    var virtualObjects = [VirtualObject]()
    
    /// The rows of the currently selected `VirtualObject`s.
    var selectedVirtualObjectRows = IndexSet()
    
    weak var delegate: ViewController?
    
    @objc func selectedMenuItem(_ menuItem: NSMenuItem) {
        guard let object = menuItem.representedObject as? VirtualObject else {
            return
        }
        
        // Check if the current row is already selected, then deselect it.
        delegate?.virtualObjectSelectionViewController(self, didSelectObject: object)
    }

    func openMenu(event: NSEvent, vc: ViewController) {
        if (virtualObjects.isEmpty) {
            virtualObjects = VirtualObject.availableObjects
        }
        self.delegate = vc
        let theMenu = NSMenu(title: "Create Object")
        var index = 0
        for virtualObject in virtualObjects {
            let menuItem = NSMenuItem(title: virtualObject.modelName, action: #selector(selectedMenuItem(_:)), keyEquivalent: "")
            menuItem.representedObject = virtualObject
            menuItem.target = self
            theMenu.insertItem(menuItem, at: index)
            index += 1
        }
        NSMenu.popUpContextMenu(theMenu, with: event, for: vc.view)
    }
}

public typealias VirtualObjectSelectionViewController = MacVirtualObjectSelection

