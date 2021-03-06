/*
See LICENSE folder for this sample’s licensing information.

Abstract:
A custom `ARSCNView` configured for the requirements of this project.
*/

import Foundation
import ARKit

extension ARSCNView {

    // MARK: Position Testing
    
    func smartHitTest(_ point: CGPoint,
                      infinitePlane: Bool = false,
                      objectPosition: float3? = nil,
                      allowedAlignments: [ARPlaneAnchor.Alignment] = [.horizontal]) -> ARHitTestResult? {
		
		// Perform the hit test.
        let existingPlaneType = ARHitTestResult.ResultType.existingPlaneUsingGeometry;
		let results = hitTest(point, types: [existingPlaneType, .estimatedVerticalPlane, .estimatedHorizontalPlane])

		// 1. Check for a result on an existing plane using geometry.
        if let existingPlaneUsingGeometryResult = results.first(where: { $0.type == existingPlaneType }),
            let planeAnchor = existingPlaneUsingGeometryResult.anchor as? ARPlaneAnchor, allowedAlignments.contains(planeAnchor.alignment) {
            return existingPlaneUsingGeometryResult
		}
		
		if infinitePlane {
			
			// 2. Check for a result on an existing plane, assuming its dimensions are infinite.
			//    Loop through all hits against infinite existing planes and either return the
			//    nearest one (vertical planes) or return the nearest one which is within 5 cm
			//    of the object's position.
			let infinitePlaneResults = hitTest(point, types: .existingPlane)
            
            for infinitePlaneResult in infinitePlaneResults {
                if let planeAnchor = infinitePlaneResult.anchor as? ARPlaneAnchor, allowedAlignments.contains(planeAnchor.alignment) {
                    if planeAnchor.alignment == .vertical {
                        // Return the first vertical plane hit test result.
                        return infinitePlaneResult
                    }
                    // For horizontal planes we only want to return a hit test result
                    // if it is close to the current object's position.
                    if let objectY = objectPosition?.y {
                        let planeY = infinitePlaneResult.worldTransform.translation.y
                        if objectY > planeY - 0.05 && objectY < planeY + 0.05 {
                            return infinitePlaneResult
                        }
                    } else {
                        return infinitePlaneResult
                    }
                }
            }
		}
		
		// 3. As a final fallback, check for a result on estimated planes.
		let vResult = results.first(where: { $0.type == .estimatedVerticalPlane })
		let hResult = results.first(where: { $0.type == .estimatedHorizontalPlane })
        let hasVertical = allowedAlignments.contains(.vertical)
        switch (allowedAlignments.contains(.horizontal), hasVertical) {
            case (true, false):
                return hResult
            case (false, true):
                // Allow fallback to horizontal because we assume that objects meant for vertical placement
                // (like a picture) can always be placed on a horizontal surface, too.
                return vResult ?? hResult
            case (true, true):
                if hResult != nil && vResult != nil {
                    return hResult!.distance < vResult!.distance ? hResult! : vResult!
                } else {
                    return hResult ?? vResult
                }
            default:
                return nil
        }
	}
	
	// - MARK: Object anchors
	/// - Tag: AddOrUpdateAnchor
	func addOrUpdateAnchor(for object: VirtualObject) {
		// If the anchor is not nil, remove it from the session.
		if let anchor = object.anchor {
			session.remove(anchor: anchor)
		}
		
		// Create a new anchor with the object's current transform and add it to the session
		let newAnchor = ARAnchor(transform: object.simdWorldTransform)
		object.anchor = newAnchor
		session.add(anchor: newAnchor)
	}

#if false
    /**
     Type conversion wrapper for original `unprojectPoint(_:)` method.
     Used in contexts where sticking to SIMD float3 type is helpful.
     */
    func unprojectPoint(_ point: float3) -> float3 {
        return float3(unprojectPoint(SCNVector3(point)))
    }
#endif
}

