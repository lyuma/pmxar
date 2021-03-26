/*
See LICENSE folder for this sample’s licensing information.

Abstract:
A `SCNReferenceNode` subclass for virtual objects placed into the AR scene.
*/

import Foundation
import SceneKit
#if os(iOS) || os(watchOS)
import ARKit
public typealias HMPlaneAnchorAlignment = ARPlaneAnchor.Alignment
#else
public enum HorizontalAlignment {
    case horizontal
}
public typealias HMPlaneAnchorAlignment = HorizontalAlignment
public typealias ARAnchor = NSObject
public typealias ARPlaneAnchor = NSObject
public typealias ARHitTestResult = NSObject
#endif
import os.log

extension SCNView {
    /// Hit tests against the `sceneView` to find an object at the provided point.
    func virtualObject(at point: CGPoint) -> VirtualObject? {
        let hitTestOptions: [SCNHitTestOption: Any] = [.boundingBoxOnly: true]
        let hitTestResults = hitTest(point, options: hitTestOptions)
        
        return hitTestResults.lazy.flatMap { result in
            return VirtualObject.existingObjectContainingNode(result.node)
        }.first
    }
}

class VirtualObject: SCNNode {
    
    var referenceURL: URL
    var mmdNode : SCNNode = SCNNode()
    var scnGeometry : SCNGeometry?
    var scnSkinner : SCNSkinner?
    var scnMorpher : SCNMorpher?
    var geomLoader : MMDGeometryLoader?
    var vmdLoader : MMDMotionLoader?
    var vmdController : MMDMotionController?
    
    var initTime : TimeInterval {
        set {
            vmdController?.initTime = newValue;
        }
        get {
            return vmdController!.initTime;
        }
    }
    var curFrame : Float {
        set {
            vmdController?.curFrame = newValue;
        }
        get {
            return vmdController!.curFrame;
        }
    }
    var motionPaused : Bool {
        set {
            vmdController?.paused = newValue;
        }
        get {
            return vmdController!.paused;
        }
    }
    
    var showRigidBodies: Bool {
        set {
            geomLoader?.showRigidBodies = newValue;
        }
        get {
            return geomLoader!.showRigidBodies;
        }
    }
    var showConstraints: Bool {
        set {
            geomLoader?.showConstraints = newValue;
        }
        get {
            return geomLoader!.showConstraints;
        }
    }
    var showConstraintLimits: Bool {
        set {
            geomLoader?.showConstraintLimits = newValue;
        }
        get {
            return geomLoader!.showConstraintLimits;
        }
    }
    var showText: Bool {
        set {
            geomLoader?.showText = newValue;
        }
        get {
            return geomLoader!.showText;
        }
    }
    var debugCollision: Bool {
        set {
            geomLoader?.debugCollision = newValue;
        }
        get {
            return geomLoader!.debugCollision;
        }
    }
    /*var otherDebug: Bool {
        set {
            geomLoader?.otherDebug = newValue;
        }
        get {
            return geomLoader!.otherDebug;
        }
    }*/
    var geometryHidden: Bool {
        set {
            if (newValue) {
                mmdNode.skinner = nil;
                mmdNode.morpher = nil;
                mmdNode.geometry = nil;
                mmdNode.geometry = SCNBox(width: 0.01, height: 0.01, length: 0.01, chamferRadius: 0);
            } else {
                mmdNode.geometry = nil;
                mmdNode.skinner = scnSkinner;
                mmdNode.morpher = scnMorpher;
                mmdNode.geometry = scnGeometry;
            }
        }
        get {
            return mmdNode.morpher == nil;
        }
    }

    // dunno why I need this now
    required init?(coder aDecoder: NSCoder) {
        fatalError("use init(URL) method")
        //super.init(coder: aDecoder)
    }

    init(_ url: URL) {
        self.referenceURL = url
        super.init()
        self.name = url.lastPathComponent
        addChildNode(mmdNode)
    }

    public func tick(_ time: TimeInterval) {
        if (mmdNode.geometry != nil) {
            geomLoader?.tick(time);
            vmdController?.tick(time);
        }
    }

    func load() {
        if (geomLoader != nil) {
            return; // REfuse to load duplicates for now.
        }
        geomLoader = MMDGeometryLoader(fromFile:referenceURL.path)
        if (geomLoader == nil) {
            let log = OSLog(subsystem: Bundle.main.bundleIdentifier!, category: "network")
            os_log("Failed to load URL %@", log: log, referenceURL.absoluteString)
            return;
        }
        mmdNode.name = "MMD Node"
        //mmdNode.opacity = 0.3
        mmdNode.scale = SCNVector3Make(0.07, 0.07, 0.07)
        mmdNode.simdOrientation = simd_quaternion(Float(M_PI), simd_float3(0,1,0))
        scnGeometry = geomLoader!.constructGeometry(with:ViewController.mainMetalDevice())
        scnSkinner = geomLoader!.constructSkinner(from:scnGeometry, attachedTo:mmdNode)
        scnMorpher = geomLoader!.constructMorpher()
        mmdNode.skinner = scnSkinner
        mmdNode.morpher = scnMorpher
        mmdNode.geometry = scnGeometry
        //mmdNode.addChildNode(geomLoader!.rootNode)
        //vmdLoader = MMDMotionLoader(fromFile:Bundle.main.path(forResource:"ノーマルTda式用", ofType:"vmd", inDirectory:"vmd/桃源恋歌配布用motion"));
        vmdLoader = MMDMotionLoader(fromFile:Bundle.main.path(forResource:"lamb足ボーン短い人用", ofType:"vmd", inDirectory:"vmd/lamb"));
        //vmdLoader = MMDMotionLoader(fromFile:Bundle.main.path(forResource:"stand2", ofType:"vmd", inDirectory:"vmd/stand"));
        vmdController = vmdLoader!.buildModelControllerFronmGeomController(geomLoader);
        //vmdController?.tick(0);
    }
    
    /// The model name derived from the `referenceURL`.
    var modelName: String {
        return referenceURL.lastPathComponent.replacingOccurrences(of: ".scn", with: "")
    }
    
    /// Use average of recent virtual object distances to avoid rapid changes in object scale.
    private var recentVirtualObjectDistances = [Float]()
	
	/// Allowed alignments for the virtual object
	var allowedAlignments: [HMPlaneAnchorAlignment] {
		return [.horizontal]
	}
	
	/// Current alignment of the virtual object
	var currentAlignment: HMPlaneAnchorAlignment = .horizontal
	
	/// Whether the object is currently changing alignment
	private var isChangingAlignment: Bool = false
	
	/// For correct rotation on horizontal and vertical surfaces, roate around
	/// local y rather than world y. Therefore rotate first child note instead of self.
	var objectRotation: HMSCNFloat {
		get {
			return childNodes.first!.eulerAngles.y
		}
		set (newValue) {
			var normalized = newValue.truncatingRemainder(dividingBy: 2 * .pi)
			normalized = (normalized + 2 * .pi).truncatingRemainder(dividingBy: 2 * .pi)
			if normalized > .pi {
				normalized -= 2 * .pi
			}
			childNodes.first!.eulerAngles.y = HMSCNFloat(normalized)
			if currentAlignment == .horizontal {
				rotationWhenAlignedHorizontally = normalized
			}
		}
	}
	
	/// Remember the last rotation for horizontal alignment
	var rotationWhenAlignedHorizontally: HMSCNFloat = 0
	
	/// The object's corresponding ARAnchor
	var anchor: ARAnchor?
    
    /// Resets the object's position smoothing.
    func reset() {
        recentVirtualObjectDistances.removeAll()
    }

#if os(iOS) || os(watchOS)
	// MARK: - Helper methods to determine supported placement options
	func isPlacementValid(on planeAnchor: ARPlaneAnchor?) -> Bool {
		if let anchor = planeAnchor {
			return allowedAlignments.contains(anchor.alignment)
		}
		return true
	}
#endif

    /**
     Set the object's position based on the provided position relative to the `cameraTransform`.
     If `smoothMovement` is true, the new position will be averaged with previous position to
     avoid large jumps.
     
     - Tag: VirtualObjectSetPosition
     */
    func setTransform(_ newTransform: float4x4,
                      relativeTo cameraTransform: float4x4,
                      smoothMovement: Bool,
                      alignment: HMPlaneAnchorAlignment,
                      allowAnimation: Bool) {
        let cameraWorldPosition = cameraTransform.translation
        var positionOffsetFromCamera = newTransform.translation - cameraWorldPosition
        
        // Limit the distance of the object from the camera to a maximum of 10 meters.
        if simd_length(positionOffsetFromCamera) > 10 {
            positionOffsetFromCamera = simd_normalize(positionOffsetFromCamera)
            positionOffsetFromCamera *= 10
        }
        
        /*
         Compute the average distance of the object from the camera over the last ten
         updates. Notice that the distance is applied to the vector from
         the camera to the content, so it affects only the percieved distance to the
         object. Averaging does _not_ make the content "lag".
         */
        if smoothMovement {
            let hitTestResultDistance = simd_length(positionOffsetFromCamera)
            
            // Add the latest position and keep up to 10 recent distances to smooth with.
            recentVirtualObjectDistances.append(hitTestResultDistance)
            recentVirtualObjectDistances = Array(recentVirtualObjectDistances.suffix(10))
            
            let averageDistance = recentVirtualObjectDistances.average!
            let averagedDistancePosition = simd_normalize(positionOffsetFromCamera) * averageDistance
            simdPosition = cameraWorldPosition + averagedDistancePosition
        } else {
            simdPosition = cameraWorldPosition + positionOffsetFromCamera
        }
		
		updateAlignment(to: alignment, transform: newTransform, allowAnimation: allowAnimation)
    }
	
	// MARK: - Setting the object's alignment
	
	func updateAlignment(to newAlignment: HMPlaneAnchorAlignment, transform: float4x4, allowAnimation: Bool) {
		if isChangingAlignment {
			return
		}
		
		// Only animate if the alignment has changed.
		let animationDuration = (newAlignment != currentAlignment && allowAnimation) ? 0.5 : 0
		
		var newObjectRotation: HMSCNFloat?
		if newAlignment == .horizontal {// && currentAlignment == .vertical {
			// When changing to horizontal placement, restore the previous horizontal rotation.
			newObjectRotation = rotationWhenAlignedHorizontally
		} else if currentAlignment == .horizontal { // && newAlignment == .vertical {
			// When changing to vertical placement, reset the object's rotation (y-up).
			newObjectRotation = HMSCNFloat(0.0001)
		}
		
		currentAlignment = newAlignment
		
		SCNTransaction.begin()
		SCNTransaction.animationDuration = animationDuration
		SCNTransaction.completionBlock = {
			self.isChangingAlignment = false
		}
		
		isChangingAlignment = true
		
		// Use the filtered position rather than the exact one from the transform.
		simdTransform = transform
		simdTransform.translation = simdWorldPosition
		
		if newObjectRotation != nil {
			objectRotation = newObjectRotation!
		}
		
		SCNTransaction.commit()
	}
    
#if os(iOS) || os(watchOS)
    /// - Tag: AdjustOntoPlaneAnchor
    func adjustOntoPlaneAnchor(_ anchor: ARPlaneAnchor, using node: SCNNode) {
		// Test if the alignment of the plane is compatible with the object's allowed placement
		if !allowedAlignments.contains(anchor.alignment) {
			return
		}
		
		// Get the object's position in the plane's coordinate system.
        let planePosition = node.convertPosition(position, from: parent)
        
        // Check that the object is not already on the plane.
        guard planePosition.y != 0 else { return }
		
        // Add 10% tolerance to the corners of the plane.
        let tolerance: Float = 0.1
        
        let minX: Float = anchor.center.x - anchor.extent.x / 2 - anchor.extent.x * tolerance
        let maxX: Float = anchor.center.x + anchor.extent.x / 2 + anchor.extent.x * tolerance
        let minZ: Float = anchor.center.z - anchor.extent.z / 2 - anchor.extent.z * tolerance
        let maxZ: Float = anchor.center.z + anchor.extent.z / 2 + anchor.extent.z * tolerance
        
        guard (minX...maxX).contains(planePosition.x) && (minZ...maxZ).contains(planePosition.z) else {
            return
        }
        
        // Move onto the plane if it is near it (within 5 centimeters).
        let verticalAllowance: Float = 0.05
        let epsilon: Float = 0.001 // Do not update if the difference is less than 1 mm.
        let distanceToPlane = abs(planePosition.y)
        if distanceToPlane > epsilon && distanceToPlane < verticalAllowance {
            SCNTransaction.begin()
            SCNTransaction.animationDuration = CFTimeInterval(distanceToPlane * 500) // Move 2 mm per second.
            SCNTransaction.animationTimingFunction = CAMediaTimingFunction(name: kCAMediaTimingFunctionEaseInEaseOut)
            position.y = anchor.transform.columns.3.y
			updateAlignment(to: anchor.alignment, transform: simdWorldTransform, allowAnimation: false)
            SCNTransaction.commit()
        }
    }
#endif
}

extension VirtualObject {
    // MARK: Static Properties and Methods
    
    /// Loads all the model objects within `Models.scnassets`.
    static let availableObjects: [VirtualObject] = {
    /*
        let modelsURL = Bundle.main.url(forResource: "Models.scnassets", withExtension: nil)!

        let fileEnumerator = FileManager().enumerator(at: modelsURL, includingPropertiesForKeys: [])!

        return fileEnumerator.flatMap { element in
            let url = element as! URL

            guard url.pathExtension == "scn" else { return nil }

            return VirtualObject(url: url)
        }
    */
        let modelsURL = Bundle.main.url(forResource: "pmx", withExtension: nil)!
        let fileEnumerator = FileManager().enumerator(at: modelsURL, includingPropertiesForKeys: [])!
        var ret : [VirtualObject] = fileEnumerator.flatMap { element in
            let url = element as! URL

            guard url.pathExtension == "pmx" || url.pathExtension == "pmd" else { return nil }

            return VirtualObject(url)
        }
        return ret;
    }()
    
    /// Returns a `VirtualObject` if one exists as an ancestor to the provided node.
    static func existingObjectContainingNode(_ node: SCNNode) -> VirtualObject? {
        if let virtualObjectRoot = node as? VirtualObject {
            return virtualObjectRoot
        }
        
        guard let parent = node.parent else { return nil }
        
        // Recurse up to check if the parent is a `VirtualObject`.
        return existingObjectContainingNode(parent)
    }
}

extension Collection where Element == Float, Index == Int {
    /// Return the mean of a list of Floats. Used with `recentVirtualObjectDistances`.
    var average: Float? {
        guard !isEmpty else {
            return nil
        }

        let sum = reduce(Float(0)) { current, next -> Float in
            return current + next
        }
        
        return sum / Float(endIndex - startIndex)
    }
}
