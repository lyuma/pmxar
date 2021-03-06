/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Methods on the main view controller for handling virtual object loading and movement
*/

#if os(iOS) || os(watchOS)
import UIKit
#else
import Cocoa
#endif
import SceneKit

extension ViewController: VirtualObjectSelectionViewControllerDelegate {
    /**
     Adds the specified virtual object to the scene, placed using
     the focus square's estimate of the world-space position
     currently corresponding to the center of the screen.
     
     - Tag: PlaceVirtualObject
     */
    func placeVirtualObject(_ virtualObject: VirtualObject) {
        guard let cameraTransform = virtualObjectInteraction?.cameraTransform else {
#if os(iOS) || os(watchOS)
             statusViewController!.showMessage("CANNOT PLACE OBJECT\nTry moving left or right.")
#endif
             return
        }
        virtualObjectInteraction!.selectedObject = virtualObject
        let newObjTransform = virtualObjectInteraction!.hitTestPosition(screenCenter, infinitePlane: true, objectPosition: nil)
		virtualObject.setTransform(newObjTransform,
								   relativeTo: cameraTransform,
								   smoothMovement: false,
								   alignment: .horizontal,
								   allowAnimation: false)
        
        updateQueue.async {
            self.sceneView.scene!.rootNode.addChildNode(virtualObject)
#if os(iOS) || os(watchOS)
			self.arSceneView?.addOrUpdateAnchor(for: virtualObject)
#endif
        }
    }
    
    // MARK: - VirtualObjectSelectionViewControllerDelegate
    
    func virtualObjectSelectionViewController(_: VirtualObjectSelectionViewController, didSelectObject object: VirtualObject) {
        virtualObjectLoader.loadVirtualObject(object, loadedHandler: { [unowned self] loadedObject in
            DispatchQueue.main.async {
                self.hideObjectLoadingUI()
                self.placeVirtualObject(loadedObject)
            }
        })

        displayObjectLoadingUI()
    }
    
    func virtualObjectSelectionViewController(_: VirtualObjectSelectionViewController, didDeselectObject object: VirtualObject) {
        guard let objectIndex = virtualObjectLoader.loadedObjects.index(of: object) else {
            fatalError("Programmer error: Failed to lookup virtual object in scene.")
        }
        virtualObjectLoader.removeVirtualObject(at: objectIndex)
		virtualObjectInteraction!.selectedObject = nil
#if os(iOS) || os(watchOS)
		if let anchor = object.anchor {
			session?.remove(anchor: anchor)
		}
#endif
    }

    // MARK: Object Loading UI

    func displayObjectLoadingUI() {
        // Show progress indicator.
        spinner.startAnimating()
        
        addObjectButton.setImage(#imageLiteral(resourceName: "buttonring"), for: [])

        addObjectButton.isEnabled = false
        isRestartAvailable = false
    }

    func hideObjectLoadingUI() {
        // Hide progress indicator.
        spinner.stopAnimating()

        addObjectButton.setImage(#imageLiteral(resourceName: "add"), for: [])
        addObjectButton.setImage(#imageLiteral(resourceName: "addPressed"), for: [.highlighted])

        addObjectButton.isEnabled = true
        isRestartAvailable = true
    }
}
