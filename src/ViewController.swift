/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Main view controller for the AR experience.
*/

import SceneKit

#if os(iOS) || os(watchOS)
    import ARKit
    import UIKit
    public typealias HMFont=UIFont
    public typealias HMColor=UIColor
    public typealias HMViewController=UIViewController
    public typealias HMView=UIView
    public typealias HMImage=UIImage
    public typealias HMButton=UIButton
    public typealias HMVisualEffectView=UIVisualEffectView
    public typealias HMActivityIndicatorView=UIActivityIndicatorView

    public protocol HMPasteboardReading { }
    public protocol HMPasteboardWriting { }
  
//#elseif os(OSX)
#else
    import Cocoa
    public typealias HMFont=NSFont
    public typealias HMColor=NSColor
    public typealias HMViewController=NSViewController
    public typealias HMView=NSView
    public typealias HMImage=NSImage
    public typealias HMButton=NSButton
    public typealias HMVisualEffectView=NSVisualEffectView
    public enum HMButtonState {
        case highlighted
    }
    extension NSButton {
        public func setImage(_ image : NSImage, for forList: [HMButtonState]) {
            if (forList.isEmpty) {
                self.image = image
            }
        }
    }
    public class ActivityIndicatorView: NSProgressIndicator {
        public override init(frame frameRect: NSRect) {
            super.init(frame: frameRect)
            self.style = NSProgressIndicatorStyle.spinningStyle
        }
        public required init?(coder: NSCoder) {
            super.init(coder: coder);
        }
        
        public func startAnimating() {
            self.isHidden = false
            super.startAnimation(nil)
        }
        
        public func stopAnimating() {
            super.stopAnimation(nil)
            self.isHidden = true
        }
    }
    public typealias HMActivityIndicatorView=ActivityIndicatorView

    public typealias HMPasteboardReading=NSPasteboardReading
    public typealias HMPasteboardWriting=NSPasteboardWriting
  
#endif

class ViewController: HMViewController {
    
    // MARK: IBOutlets

    @IBOutlet var sceneView: SCNView!

    @IBOutlet weak var addObjectButton: HMButton!
    
    @IBOutlet weak var blurView: HMVisualEffectView!
    
    @IBOutlet weak var spinner: HMActivityIndicatorView!

    // MARK: - UI Elements
    
    /// The view controller that displays the status and "restart experience" UI.
#if os(iOS) || os(watchOS)
    var arSceneView: ARSCNView?

    var statusViewController: StatusViewController?

    let ENABLE_AR = true

    /// Convenience accessor for the session owned by ARSCNView.
    var session: ARSession? {
        return arSceneView?.session
    }
    
    /// The view controller that displays the virtual object selection menu.
    var objectsViewController: VirtualObjectSelectionViewController?
#else
    let ENABLE_AR = false
    var arSceneView: NSObject? = nil
    var statusViewController: NSObject? = nil
    var session: NSObject? = nil

    let macVirtualObjectSelection = MacVirtualObjectSelection()

    @IBAction func showVirtualObjectSelectionViewController(_ sender: HMView) {
        macVirtualObjectSelection.openMenu(event: NSApp.currentEvent!, vc: self)
    }
#endif
    
    // MARK: - ARKit Configuration Properties
    
    /// A type which manages gesture manipulation of virtual content in the scene.
    var virtualObjectInteraction: VirtualObjectInteraction?
    
    /// Coordinates the loading and unloading of reference nodes for virtual objects.
    let virtualObjectLoader = VirtualObjectLoader()
    
    /// Marks if the AR experience is available for restart.
    var isRestartAvailable = true
    
    /// A serial queue used to coordinate adding or removing nodes from the scene.
    let updateQueue = DispatchQueue(label: "com.example.apple-samplecode.arkitexample.serialSceneKitQueue")
    
    static var mainSceneRenderer : SCNSceneRenderer?
    
    static func mainMetalDevice() -> MTLDevice? {
        #if (arch(i386) || arch(x86_64))
            return nil
        #else
            return mainSceneRenderer?.device
        #endif
    }
    
    var screenCenter: CGPoint {
        if (arSceneView != nil) {
            let bounds = sceneView.bounds
            return CGPoint(x: bounds.midX, y: bounds.midY)
        } else {
            let bounds = sceneView.bounds
            return CGPoint(x: bounds.midX, y: (bounds.midY) / 2)
        }
    }
    
    // MARK: - View Controller Life Cycle

#if os(iOS) || os(watchOS)
    override func viewDidLoad() {
        handleViewDidLoad()
    }
#else
    override func awakeFromNib() {
        // Supposedly more reliable than viewDidLoad on macOS
        // I still see white screen about 20% of the time.
        handleViewDidLoad()
    }
#endif

    func handleViewDidLoad() {
        super.viewDidLoad()

#if os(iOS) || os(watchOS)
        if (ENABLE_AR && ARWorldTrackingConfiguration.isSupported) {
            let superview = sceneView.superview!
            let arView = ARSCNView(frame:self.view.bounds)
            arView.autoresizingMask = [UIViewAutoresizing.flexibleWidth, UIViewAutoresizing.flexibleHeight]
            superview.insertSubview(arView, aboveSubview: sceneView);
            sceneView.removeFromSuperview();
            sceneView = arView;
        }
        arSceneView = sceneView as? ARSCNView
        arSceneView?.session.delegate = self
        statusViewController = childViewControllers.lazy.flatMap({ $0 as? StatusViewController }).first!
        arSceneView?.automaticallyUpdatesLighting = false
#endif
        sceneView.delegate = self
        addObjectButton.isHidden = false
        if (self.arSceneView == nil) {
            self.sceneView.backgroundColor = HMColor(red: 0.8, green: 0.7, blue: 0.9, alpha: 1.0)
        }
        //sceneView.debugOptions = [ARSCNDebugOptions.showFeaturePoints, ARSCNDebugOptions.showWorldOrigin]
        //sceneView.debugOptions = [SCNDebugOptions.renderAsWireframe]
        ViewController.mainSceneRenderer = sceneView;

        virtualObjectInteraction = VirtualObjectInteraction(sceneView: sceneView)

        // Set up scene content.
        setupCamera()

        if (self.arSceneView == nil) {
            let ambientLight : SCNLight = SCNLight();
            ambientLight.color = HMColor.darkGray;
            ambientLight.type = SCNLight.LightType.ambient;
            ambientLight.intensity = CGFloat(0.2)
            let lightNode : SCNNode = SCNNode()
            lightNode.light = ambientLight;
            lightNode.name = "and He said, let there be Light!"
            self.sceneView.scene?.rootNode.addChildNode(lightNode)
        }

        /*
         The `sceneView.automaticallyUpdatesLighting` option creates an
         ambient light source and modulates its intensity. This sample app
         instead modulates a global lighting environment map for use with
         physically based materials, so disable automatic lighting.
         */
        if let environmentMap = HMImage(named: "Models.scnassets/sharedImages/environment_blur.exr") {
            sceneView.scene!.lightingEnvironment.contents = environmentMap
        }

#if os(iOS) || os(watchOS)
        // Hook up status view controller callback(s).
        statusViewController!.restartExperienceHandler = { [unowned self] in
            self.restartExperience()
        }
    
        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(showVirtualObjectSelectionViewController))
        // Set the delegate to ensure this gesture is only used when there are no virtual objects in the scene.
        tapGesture.delegate = self
        sceneView.addGestureRecognizer(tapGesture)
#endif
    }

#if os(iOS) || os(watchOS)
	override func viewDidAppear(_ animated: Bool) {
		super.viewDidAppear(animated)
		
		// Prevent the screen from being dimmed to avoid interuppting the AR experience.
		UIApplication.shared.isIdleTimerDisabled = true

        // Start the `ARSession`.
        resetTracking()
	}
	
	override func viewWillDisappear(_ animated: Bool) {
		super.viewWillDisappear(animated)

        session?.pause()
	}
#endif

    func pause() {
        sceneView.scene?.isPaused = true
    }

    func resume() {
        sceneView.scene?.isPaused = false
        sceneView.play(self)
#if !os(iOS) && !os(watchOS)
        sceneView.needsDisplay = true
#endif
        sceneView.preferredFramesPerSecond = 60
        sceneView.sceneTime += 1.0
    }
    
    func setTarget(_ target : NSObject) {
        print("New target:", target)
    }

    // MARK: - Scene content setup

    func setupCamera() {
        sceneView.showsStatistics = true;
        sceneView.antialiasingMode = .none
        if sceneView.scene == nil {
            let newScene : SCNScene = SCNScene()
            sceneView.scene = newScene
            let cameraNode = SCNNode()
            cameraNode.name = "and the Camera did peer into Schrodinger's Scene"
            cameraNode.camera = SCNCamera()
            cameraNode.camera!.zNear = 0.01
            //cameraNode.look(at: SCNVector3Make(0, 0.25, 0))
            cameraNode.position = SCNVector3Make(0, 0.75, 2)
            newScene.rootNode.addChildNode(cameraNode)
            sceneView.pointOfView = cameraNode
            sceneView.allowsCameraControl = true;
            // Create a reflective floor and configure it
            let xfloor : SCNFloor = SCNFloor()
            xfloor.reflectionFalloffEnd = 100.0;                                                    // Set a falloff end value for the reflection
            xfloor.firstMaterial!.diffuse.contents = HMImage(named:"floor.jpg");              // Set a diffuse texture, here a pavement image
            xfloor.firstMaterial!.diffuse.contentsTransform = SCNMatrix4MakeScale(40, 40, 40); // Scale the diffuse texture
            //xfloor.firstMaterial.diffuse.contentsTransform = CATransform3DMakeScale(0.4, 0.4, 0.4); // Scale the diffuse texture
            xfloor.firstMaterial!.diffuse.mipFilter = SCNFilterMode.nearest;                            // Turn on mipmapping for the diffuse texture for a better antialiasing
            xfloor.firstMaterial!.transparency = 0.5
            // Create a node to attach the floor to, and add it to the scene
            let floorNode : SCNNode = SCNNode();
            floorNode.name = "and on the seventh line of code, there was Floor";
            floorNode.geometry = xfloor;
            newScene.rootNode.addChildNode(floorNode);
        }
        guard let camera = sceneView.pointOfView?.camera else {
            assert(false && "No pointOfView in sceneView.".count == 0)
            return;
        }

        /*
         Enable HDR camera settings for the most realistic appearance
         with environmental lighting and physically based materials.
         */
        camera.wantsHDR = true
        camera.exposureOffset = -1
        camera.minimumExposure = -1
        camera.maximumExposure = 3
    }

    // MARK: - Session management
    
    /// Creates a new AR configuration to run on the `session`.
	func resetTracking() {
        guard let sess = session else {
            return;
        }
		virtualObjectInteraction!.selectedObject = nil
#if os(iOS) || os(watchOS)
        let configuration = ARWorldTrackingConfiguration()
        configuration.planeDetection = [.horizontal, .vertical]
		sess.run(configuration, options: [.resetTracking, .removeExistingAnchors])

        statusViewController!.scheduleMessage("FIND A SURFACE TO PLACE AN OBJECT", inSeconds: 7.5, messageType: .planeEstimation)
#endif
	}

    // MARK: - Focus Square

	func updateFocusSquare() {
        let isObjectVisible = virtualObjectLoader.loadedObjects.contains { object in
            return sceneView.isNode(object, insideFrustumOf: sceneView.pointOfView!)
        }
        
#if os(iOS) || os(watchOS)
        if !isObjectVisible {
            statusViewController!.scheduleMessage("TRY MOVING LEFT OR RIGHT", inSeconds: 5.0, messageType: .focusSquare)
        }
#endif

        addObjectButton.isHidden = false

/*
		if let result = self.sceneView.smartHitTest(screenCenter) {
            virtualObjectInteraction.setHitTestLocation(result);
			updateQueue.async {
				//self.sceneView.scene.rootNode.addChildNode(self.focusSquare)
				let camera = self.session.currentFrame?.camera
				//self.focusSquare.state = .detecting(hitTestResult: result, camera: camera)
			}
		} else {
			return
		}
        virtualObjectInteraction.setHitTestLocation(result);
*/
        virtualObjectInteraction!.hitTestPosition(screenCenter, infinitePlane: true, objectPosition: nil)

#if os(iOS) || os(watchOS)
        statusViewController!.cancelScheduledMessage(for: .focusSquare)
#endif
	}
    
	// MARK: - Error handling
    
    func displayErrorMessage(title: String, message: String) {
        // Blur the background.
        blurView.isHidden = false
#if os(iOS) || os(watchOS)
        // Present an alert informing about the error that has occurred.
        let alertController = UIAlertController(title: title, message: message, preferredStyle: .alert)
        let restartAction = UIAlertAction(title: "Restart Session", style: .default) { _ in
            alertController.dismiss(animated: true, completion: nil)
            self.blurView.isHidden = true
            self.resetTracking()
        }
        alertController.addAction(restartAction)
        present(alertController, animated: true, completion: nil)
#else
        let alert = NSAlert()
        alert.messageText = title
        alert.informativeText = message
        alert.runModal()
#endif
    }

}
