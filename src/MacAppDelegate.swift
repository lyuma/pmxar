/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Application's delegate.
*/

import Cocoa

@NSApplicationMain
class MacAppDelegate: NSResponder, NSApplicationDelegate {
	@IBOutlet weak var window: NSWindow!

    @IBAction func showMainWindow(_ sender: NSObject)
    {
        _ = self.applicationShouldHandleReopen(NSApplication.shared(), hasVisibleWindows: true);
        NSApplication.shared().activate(ignoringOtherApps: true)
    }

    public func applicationDidFinishLaunching(_ notification: Notification) {
        print("Finished launching")
        let vc = ViewController()
        window.contentViewController = vc
        window.contentView = vc.view
        window.isReleasedWhenClosed = false
        if let viewController = self.window.contentViewController as? ViewController {
            viewController.pause()
        }
        window.makeKeyAndOrderFront(NSApplication.shared())
        window.setIsVisible(true)
    }

    public func applicationShouldHandleReopen(_ sender: NSApplication, hasVisibleWindows flag: Bool) -> Bool {
        window.setIsVisible(true)
        return true
    }

	public func applicationWillResignActive(_ notification: Notification) {
		if let viewController = self.window.contentViewController as? ViewController {
            print("Pausing")
			viewController.pause()
		}
	}
	
	public func applicationDidBecomeActive(_ notification: Notification) {
		if let viewController = self.window.contentViewController as? ViewController {
            print("Resuming")
            viewController.resume()
		}
	}
}
