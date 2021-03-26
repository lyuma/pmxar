/*
See LICENSE folder for this sample’s licensing information.

Abstract:
ARSCNViewDelegate interactions for `ViewController`.
*/

import SceneKit

extension ViewController: SCNSceneRendererDelegate {
    
    // MARK: - SCNViewDelegate

    public func renderer(_ renderer: SCNSceneRenderer, willRenderScene scene: SCNScene, atTime time: TimeInterval) {
        glPointSize(5);
        virtualObjectLoader.tick(time);
    }

}
