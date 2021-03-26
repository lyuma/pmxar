/*
pmxar (MMDGeometryLoader.h)
Copyright (c) 2018 Lyuma

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#import <Foundation/Foundation.h>
#import <SceneKit/SceneKit.h>
#import <Metal/Metal.h>

@interface MMDGeometryLoader : NSObject
@property (nonatomic, readonly) NSString *filename;
@property (nonatomic, readonly) SCNNode *rootNode;
@property (nonatomic, readwrite) BOOL showRigidBodies;
@property (nonatomic, readwrite) BOOL showConstraints;
@property (nonatomic, readwrite) BOOL showConstraintLimits;
@property (nonatomic, readwrite) BOOL showText;
@property (nonatomic, readwrite) BOOL debugCollision;
@property (nonatomic, readwrite) BOOL otherDebug;

- (int)boneCount;

- (SCNNode*)boneAtIndex:(int)index;

- (instancetype)initFromFile:(NSString*)filename;

- (SCNGeometry*)constructGeometryWithDevice:(id<MTLDevice>)device;

- (SCNMorpher *)constructMorpher;

- (SCNSkinner *)constructSkinnerFromGeometry:(SCNGeometry*)geom attachedToNode:(SCNNode*)mmdNode;

- (void)animateMorphNode:(SCNNode*)node name:(NSString*)name repeatCount:(float)repeatCount;

- (void)tick:(NSTimeInterval)time;

@end

