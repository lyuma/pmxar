/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Coordinates movement and gesture interactions with virtual objects.
*/

#if os(iOS) || os(watchOS)
import UIKit
import ARKit
typealias HMPanGesture = ThresholdPanGesture
public typealias HMRotationGestureRecognizer = UIRotationGestureRecognizer
public typealias HMTapGestureRecognizer = UITapGestureRecognizer
public typealias HMGestureRecognizer = UIGestureRecognizer
public typealias HMGestureRecognizerDelegate = UIGestureRecognizerDelegate
public typealias HMSCNFloat = Float
#else
import Cocoa
extension NSPanGestureRecognizer {
    var isThresholdExceeded : Bool {
        get {
            return false
        }
    }
}
public typealias HMPanGesture = NSPanGestureRecognizer
public typealias HMRotationGestureRecognizer = NSRotationGestureRecognizer
public typealias HMTapGestureRecognizer = NSClickGestureRecognizer
public typealias HMGestureRecognizer = NSGestureRecognizer
public typealias HMGestureRecognizerDelegate = NSGestureRecognizerDelegate
public typealias HMSCNFloat = CGFloat
#endif
import SceneKit

/// - Tag: VirtualObjectInteraction
class VirtualObjectInteraction: NSObject, HMGestureRecognizerDelegate {
    
    /// Developer setting to translate assuming the detected plane extends infinitely.
    let translateAssumingInfinitePlane = true
    
    /// The scene view to hit test against when moving virtual content.
    let sceneView: SCNView!

#if os(iOS) || os(watchOS)
    let arSceneView: ARSCNView?
#else
    let arSceneView: NSObject? = nil
#endif
    
    /**
     The object that has been most recently intereacted with.
     The `selectedObject` can be moved at any time with the tap gesture.
     */
    var selectedObject: VirtualObject?
    
    var lastPlaneAnchor: ARPlaneAnchor?

    var lastY: Float?

    /// The object that is tracked for use by the pan and rotation gestures.
    private var trackedObject: VirtualObject? {
        didSet {
            guard trackedObject != nil else { return }
            selectedObject = trackedObject
        }
    }
    
    /// The tracked screen position used to update the `trackedObject`'s position in `updateObjectToCurrentTrackingPosition()`.
    private var currentTrackingPosition: CGPoint?

    init(sceneView: SCNView) {
        self.sceneView = sceneView
#if os(iOS) || os(watchOS)
        self.arSceneView = sceneView as? ARSCNView;
#endif
        super.init()
        
#if os(iOS) || os(watchOS)
        let panGesture = HMPanGesture(target: self, action: #selector(didPan(_:)))
        panGesture.delegate = self
        
        let rotationGesture = HMRotationGestureRecognizer(target: self, action: #selector(didRotate(_:)))
        rotationGesture.delegate = self
    
        // Add gestures to the `sceneView`.
        sceneView.addGestureRecognizer(panGesture)
        sceneView.addGestureRecognizer(rotationGesture)
#endif
        let tapGesture = HMTapGestureRecognizer(target: self, action: #selector(didTap(_:)))
        sceneView.addGestureRecognizer(tapGesture)
    }
    
    var cameraTransform : float4x4? {
        get {
#if os(iOS) || os(watchOS)
            if let sess = arSceneView?.session {
                return sess.currentFrame?.camera.transform
            } else {
                return sceneView.pointOfView!.simdTransform
            }
#else
            return sceneView.pointOfView!.simdTransform
#endif
        }
    }
    
    // MARK: - Gesture Actions
    
    @objc
    func didPan(_ gesture: HMPanGesture) {
        switch gesture.state {
        case .began:
            // Check for interaction with a new object.
            if let object = objectInteracting(with: gesture, in: sceneView) {
                trackedObject = object
            }
            
        case .changed where gesture.isThresholdExceeded:
            guard let object = trackedObject else { return }
            let translation = gesture.translation(in: sceneView)
            
            let currentPosition = currentTrackingPosition ?? CGPoint(sceneView.projectPoint(object.position))
            
            // The `currentTrackingPosition` is used to update the `selectedObject` in `updateObjectToCurrentTrackingPosition()`.
            currentTrackingPosition = CGPoint(x: currentPosition.x + translation.x, y: currentPosition.y + translation.y)

            gesture.setTranslation(.zero, in: sceneView)
            
        case .changed:
            // Ignore changes to the pan gesture until the threshold for displacment has been exceeded.
            break
			
		case .ended:
			// Update the object's anchor when the gesture ended.
#if os(iOS) || os(watchOS)
            guard let existingTrackedObject = trackedObject else { break }
            arSceneView?.addOrUpdateAnchor(for: existingTrackedObject)
#endif
			fallthrough
            
        default:
            // Clear the current position tracking.
            currentTrackingPosition = nil
            trackedObject = nil
        }
    }

    /**
     If a drag gesture is in progress, update the tracked object's position by
     converting the 2D touch location on screen (`currentTrackingPosition`) to
     3D world space.
     This method is called per frame (via `SCNSceneRendererDelegate` callbacks),
     allowing drag gestures to move virtual objects regardless of whether one
     drags a finger across the screen or moves the device through space.
     - Tag: updateObjectToCurrentTrackingPosition
     */
    @objc
    func updateObjectToCurrentTrackingPosition() {
        guard let object = trackedObject, let position = currentTrackingPosition else { return }
        translate(object, basedOn: position, infinitePlane: translateAssumingInfinitePlane, allowAnimation: true)
    }

    /// - Tag: didRotate
    @objc
    func didRotate(_ gesture: HMRotationGestureRecognizer) {
        guard gesture.state == .changed else { return }
        
        /*
         - Note:
          For looking down on the object (99% of all use cases), we need to subtract the angle.
          To make rotation also work correctly when looking from below the object one would have to
          flip the sign of the angle depending on whether the object is above or below the camera...
         */
        trackedObject?.objectRotation -= HMSCNFloat(gesture.rotation)
        
        gesture.rotation = 0
    }
    
    @objc
    func didTap(_ gesture: HMTapGestureRecognizer) {
        let touchLocation = gesture.location(in: sceneView)
        if let tappedObject = sceneView.virtualObject(at: touchLocation) {
            // Select a new object.
            selectedObject = tappedObject
        } else if let object = selectedObject {
            // Teleport the object to whereever the user touched the screen.
            translate(object, basedOn: touchLocation, infinitePlane: false, allowAnimation: false)
#if os(iOS) || os(watchOS)
            arSceneView?.addOrUpdateAnchor(for: object)
#endif
        }
    }
    
    func gestureRecognizer(_ gestureRecognizer: HMGestureRecognizer, shouldRecognizeSimultaneouslyWith otherGestureRecognizer: HMGestureRecognizer) -> Bool {
        // Allow objects to be translated and rotated at the same time.
        return true
    }

    /// A helper method to return the first object that is found under the provided `gesture`s touch locations.
    /// - Tag: TouchTesting
    private func objectInteracting(with gesture: HMGestureRecognizer, in view: SCNView) -> VirtualObject? {
#if os(iOS) || os(watchOS)
        for index in 0..<gesture.numberOfTouches {
            let touchLocation = gesture.location(ofTouch: index, in: view)
            
            // Look for an object directly under the `touchLocation`.
            if let object = sceneView.virtualObject(at: touchLocation) {
                return object
            }
        }
#else
        let touchLocation = gesture.location(in: view)

        // Look for an object directly under the `touchLocation`.
        if let object = sceneView.virtualObject(at: touchLocation) {
            return object
        }
#endif

        // As a last resort look for an object under the center of the touches.
        return sceneView.virtualObject(at: gesture.center(in: view))
    }
    
    // MARK: - Update object position

    @discardableResult public func hitTestPosition(_ screenPos: CGPoint, infinitePlane: Bool, objectPosition objectPos: simd_float3?) -> simd_float4x4 {
        var result : ARHitTestResult? = nil;
#if os(iOS) || os(watchOS)
        if let arView = arSceneView {
            result = arView.smartHitTest(screenPos,
                                                    infinitePlane: infinitePlane,
                                                    objectPosition: objectPos,
                                                    allowedAlignments: [.horizontal])

            var planeAlignment: HMPlaneAnchorAlignment = .horizontal
            let planeAnchor = result?.anchor as? ARPlaneAnchor
            if planeAnchor != nil {
                planeAlignment = planeAnchor!.alignment
            } else if result?.type == .estimatedHorizontalPlane {
                planeAlignment = .horizontal
            } else {
                if result?.type == .estimatedVerticalPlane {
                    planeAlignment = .vertical
                }
                result = nil;
            }
            /*if (result == nil && lastPlaneAnchor != nil) {
                let transform = lastPlaneAnchor!.worldTransform;
                let invTransform = simd_inverse(transform)
                point =
            }
            */
            if (planeAnchor != nil) {
                lastPlaneAnchor = planeAnchor
            }
        }
#endif

        var point : simd_float3
        if (result == nil) {
            let farPoint  = simd_float3(sceneView.unprojectPoint(SCNVector3Make(HMSCNFloat(screenPos.x), HMSCNFloat(screenPos.y), 1)))
            let nearPoint = simd_float3(sceneView.unprojectPoint(SCNVector3Make(HMSCNFloat(screenPos.x), HMSCNFloat(screenPos.y), 0)))
            let ray = simd_normalize(farPoint - nearPoint);
            var distance : Float
            if (arSceneView == nil && lastY == nil) {
                lastY = 0.0
            }
            if (lastY != nil) {
                distance = (lastY! - nearPoint.y) / ray.y
            } else {
                distance = 1.0
            }
            point = nearPoint + ray * distance
        } else {
#if os(iOS) || os(watchOS)
            point = result!.worldTransform.translation
#else
            point = simd_float3(0, 0, 0)
#endif
        }
        if (point.y.isFinite && point.y < 1e10 && point.y > -1e10) {
            lastY = point.y
        } else {
            NSLog("Not finite point %f %f %f", point.x, point.y, point.z);
        }
        //point = simd_float3(0,0,0)

        let transform = simd_float4x4(SCNMatrix4MakeTranslation(HMSCNFloat(point.x), HMSCNFloat(point.y), HMSCNFloat(point.z)))
        
        return transform
    }

    /// - Tag: DragVirtualObject
	private func translate(_ object: VirtualObject, basedOn screenPos: CGPoint, infinitePlane: Bool, allowAnimation: Bool) {
        if (cameraTransform == nil) {
            return;
        }
        //let cameraProjMatrix = sceneView.session.currentFrame?.camera.projectionMatrix
        let transform : simd_float4x4 = hitTestPosition(screenPos,
                                                infinitePlane: infinitePlane,
                                                objectPosition: object.simdWorldPosition)

        /*
         Plane hit test results are generally smooth. If we did *not* hit a plane,
         smooth the movement to prevent large jumps.
         */
        //let isOnPlane = result?.anchor is ARPlaneAnchor
		object.setTransform(transform,
							relativeTo: cameraTransform!,
							smoothMovement: true,
							alignment: .horizontal, // result.planeAlignment
							allowAnimation: allowAnimation)
    }
}

#if os(iOS) || os(watchOS)
/// Extends `UIGestureRecognizer` to provide the center point resulting from multiple touches.
extension HMGestureRecognizer {
    func center(in view: UIView) -> CGPoint {
        // simulator can crash with if doing multitouch...?
        // *** Terminating app due to uncaught exception 'NSRangeException', reason: '-[pmxar.ThresholdPanGesture locationOfTouch:inView:]: index (0) beyond bounds (0).'
        if (numberOfTouches == 0) {
            return CGPoint(x: 0, y: 0);
        }
        let first = CGRect(origin: location(ofTouch: 0, in: view), size: .zero)

        let touchBounds = (1..<numberOfTouches).reduce(first) { touchBounds, index in
            return touchBounds.union(CGRect(origin: location(ofTouch: index, in: view), size: .zero))
        }

        return CGPoint(x: touchBounds.midX, y: touchBounds.midY)
    }
}
#else
extension HMGestureRecognizer {
    func center(in view: NSView) -> CGPoint {
        //if (numberOfTouches == 0) {
        //    return CGPoint(x: 0, y: 0);
        //}
        let touchBounds = CGRect(origin: location(in: view), size: .zero)
        // No multitouch on mac. How to zoom? wheel?

        return CGPoint(x: touchBounds.midX, y: touchBounds.midY)
    }
}
#endif
