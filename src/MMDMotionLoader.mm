/*
pmxar (MMDMotionLoader.mm)
Copyright (c) 2018 Lyuma

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#import "MMDGeometryLoader.h"
#import "MMDMotionLoader.h"
#include "MMDVmdParser.h"
#include "TargetConditionals.h"
#include <fstream>

#include <map>

@interface MMDMotionLoader ()
- (vmd::VmdMotion &)motion;
@end

@implementation MMDMotionLoader {
    std::unique_ptr<vmd::VmdMotion> motion;
}

-(instancetype)initFromFile:(NSString*)filename {
    if ((self = [super init])) {
        _filename = filename;
        std::ifstream ifs (filename.UTF8String);
        motion = vmd::VmdMotion::LoadFromStream(&ifs);
        if (!motion.get()) {
            return nil;
        }
    }
    return self;
}

- (vmd::VmdMotion &)motion {
    return *(motion.get());
}

- (MMDMotionController*)buildModelControllerFronmGeomController:(MMDGeometryLoader*)controller {
    return [[MMDMotionController alloc] initFromLoader:self usingGeomController:controller];
}

@end

struct AnimationData {
    struct BaseFrame {
        int startFrame;
    };
    struct BoneState {
        //simd_float4x4 transform;
        simd_float3 position;
        simd_quatf orientation;
    };
    struct BoneFrame : public BaseFrame, public BoneState {
        float positionBezier[3][4];
        float quaternionBezier[4];
        
        typedef BoneState value_type;
        
        BoneFrame(int startFrame, simd_float3 position, simd_quatf orientation, const char(&bezier)[4][4])
            : BaseFrame {startFrame}, BoneState {position, orientation} {
            for (int k = 0; k < 3; k++) {
                for (int i = 0; i < 4; i++) {
                    positionBezier[k][i] = (float)bezier[k][i] / 127.f;
                }
            }
            for (int i = 0; i < 4; i++) {
                quaternionBezier[i] = (float)bezier[3][i] / 127.f;
            }
        }
        
        const value_type &value() const {
            return *this;
        }
        
        static float calculateBezier(const float (&bezier)[4], float x) {
            // From three.js
            const float x1 = bezier[0], x2 = bezier[1], y1 = bezier[2], y2 = bezier[3];
            float c = 0.5;
            float t = c;
            float s = 1.0 - t;
            constexpr int LOOP = 15;
            constexpr float EPS = 1e-5, NEPS = -1e-5;

            float sst3 = 0.f, stt3 = 0.f, ttt = 0.f;

            for ( int i = 0; i < LOOP; i ++ ) {

                sst3 = 3.0 * s * s * t;
                stt3 = 3.0 * s * t * t;
                ttt = t * t * t;

                float ft = ( sst3 * x1 ) + ( stt3 * x2 ) + ( ttt ) - x;

                if ( ft < EPS && ft > NEPS ) break;

                c /= 2.0;

                t += ( ft < 0 ) ? c : -c;
                s = 1.0 - t;

            }
            
            return ( sst3 * y1 ) + ( stt3 * y2 ) + ttt;
        }
        
        static simd_quatf slerpFlat(simd_float4 src0, simd_float4 src1, float t) {
            // From three.js
            if (simd_equal(src0, src1)) {
                return {src0};
            }
            float s = 1 - t;
            float cosx = simd_dot(src0, src1);
            int dir = (cosx >= 0 ? 1 : -1);
            float sqrSin = 1 - cosx * cosx;
            if (sqrSin > 1.e-6) {
                float sinx = sqrtf(sqrSin);
                float len = atan2(sinx, cosx * dir);
                s = sinf(s * len) / sinx;
                t = sinf(t * len) / sinx;
            }
            src0 = src0 * s + src1 * (t * dir);
            // if (s == 1 - t)
            return {simd_normalize(src0)};
        }
        
        value_type interpolate(const BoneFrame &other, float weight) const {
            float quatRatio = calculateBezier(quaternionBezier, weight);
            simd_float3 posRatio {
                    calculateBezier(positionBezier[0], weight),
                    calculateBezier(positionBezier[1], weight),
                    calculateBezier(positionBezier[2], weight)
            };
            constexpr simd_float3 one {1.f, 1.f, 1.f};
            return {
                position * (one - posRatio) + other.position * posRatio,
                //position * (1.f - weight) + other.position * weight,
                slerpFlat(orientation.vector, other.orientation.vector, quatRatio)
            };
        }
        
        bool operator== (const BoneFrame &other) const {
            return simd_equal(position, other.position) && simd_equal(orientation.vector, other.orientation.vector);
        }
        bool operator!= (const BoneFrame &other) const {
            return !(*this == other);
        }
    };
    struct MorphFrame : public BaseFrame {
        float weight;

        typedef float value_type;

        MorphFrame(int startFrame, float weight)
            : BaseFrame {startFrame}, weight(weight) {
        }
        
        value_type value() const {
            return weight;
        }
        
        value_type interpolate(const MorphFrame &other, float intweight) {
            return value() * (1.f - intweight) + other.value() * intweight;
        }
        
        bool operator== (const MorphFrame &other) const {
            return other.weight == weight;
        }
        bool operator!= (const MorphFrame &other) const {
            return !(*this == other);
        }
    };

    template <typename FrameType>
    struct PoseElementState {
        std::vector<FrameType> poses;
        size_t curPoseIndex;
        bool needsInitialize;
        
        PoseElementState()
            : curPoseIndex(0), needsInitialize(true) {
        }
        
        bool updateCurFrameAndCheckActive(float curFrame) {
            if (poses.empty()) {
                return false;
            }
            size_t prevPoseIndex = curPoseIndex;
            while (curPoseIndex > 0 && curFrame < poses[curPoseIndex].startFrame) {
                curPoseIndex--;
            }
            while (curPoseIndex + 1 < poses.size() && curFrame >= poses[curPoseIndex + 1].startFrame) {
                curPoseIndex++;
            }
            if (prevPoseIndex != curPoseIndex || needsInitialize) {
                needsInitialize = false;
                return true; // change frame = always active
            }
            if (curPoseIndex + 1 >= poses.size()) {
                return false; // end of animation = not active
            }
            // check if contents of frames differ.
            return poses[curPoseIndex] != poses[curPoseIndex + 1];
        }
        
        float lerpWeight(float curFrame) {
            if (curPoseIndex + 1 >= poses.size()) {
                return 0.f;
            }
            float prevPoseStart = poses[curPoseIndex].startFrame;
            float nextPoseStart = poses[curPoseIndex + 1].startFrame;
            if (nextPoseStart <= prevPoseStart) {
                return 0.f;
            }
            return (curFrame - prevPoseStart) / (nextPoseStart - prevPoseStart);
        }
        
        typename FrameType::value_type interpolate(float curFrame) {
            float weight = lerpWeight(curFrame);
            if (weight == 0.f || curPoseIndex + 1 >= poses.size()) {
                return poses[curPoseIndex].value();
            }
            return poses[curPoseIndex].interpolate(poses[curPoseIndex + 1], weight);
        }

        template <typename ...Params>
        auto emplace_back(Params&&... params)
        {
            return poses.emplace_back(std::forward<Params>(params)...);
        }
        
        void sortFrames() {
            std::sort(poses.begin(), poses.end(), [](auto &a, auto &b) {return a.startFrame < b.startFrame;});
        }
    };
    
    std::vector<PoseElementState<BoneFrame>> bones;
    std::vector<PoseElementState<MorphFrame>> morphs;
    /*struct IK {
        struct IKEnable {
            int ikBone;
            std::string ikName;
            bool enable;
        };
        bool display;
        std::vector<IKEnable> ik_enabme;
    };*/
    
    void init(size_t num_bones, size_t num_morphs) {
        bones.resize(num_bones);
        morphs.resize(num_morphs);
    }
    
    void addBoneFrame(int bone, const vmd::VmdBoneFrame &frame, simd_float4x4 initialTransform) {
        if (bone < 0 || bone >= bones.size()) {
            return;
        }
        /*
        simd_float4x4 transform = simd_matrix4x4(simd_quaternion(
                    frame.orientation[0], frame.orientation[1], frame.orientation[2], frame.orientation[3]));
        transform.columns[3][0] = frame.position[0];
        transform.columns[3][1] = frame.position[1];
        transform.columns[3][2] = frame.position[2];
        simd_float4x4 fullTransform = simd_mul(initialTransform, transform);
        simd_float3 pos {transform.columns[3][0], transform.columns[3][1], transform.columns[3][2]};
        simd_quatf orient = simd_quaternion(fullTransform);
        */
        simd_float3 pos = {initialTransform.columns[3][0] + frame.position[0], initialTransform.columns[3][1] + frame.position[1], initialTransform.columns[3][2] + frame.position[2]};
        simd_quatf initialOrient = simd_quaternion(initialTransform);
        simd_quatf orient = simd_mul(initialOrient, simd_quaternion(frame.orientation[0], frame.orientation[1], frame.orientation[2], frame.orientation[3]));
        bones[bone].emplace_back(frame.frame, pos, orient, frame.interpolation[0]);
    }
    
    void addMorphFrame(int morph, const vmd::VmdFaceFrame &morphFrame) {
        if (morph < 0 || morph >= morphs.size()) {
            return;
        }
        morphs[morph].emplace_back(morphFrame.frame, morphFrame.weight);
    }
    
    void sortFrames() {
        for (auto &poseState : bones) {
            poseState.sortFrames();
        }
        for (auto &poseState : morphs) {
            poseState.sortFrames();
        }
    }
};

@interface MMDMotionController ()
@end

@implementation MMDMotionController {
    SCNNode *rootNode;
    NSString *rootNodeName;
    AnimationData animationData;
    std::vector<simd_float4x4> boneInitialBindTransforms;
    std::map<std::string, int> boneNameToPhysicsNodeIdx;
    std::map<std::string, int> morphNameToIdx;

    double lastFrameTime;
    float curFrame;
    int totalFrameCount;
}

int findOrNegOne(const std::map<std::string, int>&map, const std::string &toFind) {
    auto bit = map.find(toFind);
    if (bit == map.end()) {
        return -1; // this model does not support this bone. Ignore it.
    }
    return bit->second;
}

- (instancetype)initFromLoader:(MMDMotionLoader*)loader usingGeomController:(MMDGeometryLoader*)geomController {
    if ((self = [super init])) {
        _vmdLoader = loader;
        _geomController = geomController;
        rootNode = _geomController.rootNode;
        _mmdNode = rootNode.parentNode;
        rootNode = [_mmdNode childNodeWithName:@"rootNode" recursively:NO];
        animationData.init(_geomController.boneCount, _mmdNode.morpher.targets.count);
        rootNodeName = rootNode.name;
        for (int i = 0; i < _geomController.boneCount; i++) {
            SCNNode *bone = [_geomController boneAtIndex:i];
            NSString *boneName = bone.name;
            char charstr[64];
            boneInitialBindTransforms.push_back(bone.simdTransform);//simd_inverse(bone.simdTransform));
            if (![boneName getCString:charstr maxLength:(sizeof(charstr)-1) encoding:NSShiftJISStringEncoding]) {
                NSLog(@"Failed to convert bone string %@ to sjis", boneName);
                continue;
            }
            boneNameToPhysicsNodeIdx[charstr] = i;
        }
        NSArray<SCNGeometry *> *targets = _mmdNode.morpher.targets;
        for (int i = 0; i < targets.count; i++) {
            NSString *morphName = targets[i].name;
            char charstr[64];
            if (![morphName getCString:charstr maxLength:(sizeof(charstr)-1) encoding:NSShiftJISStringEncoding]) {
                NSLog(@"Failed to convert morph string %@ to sjis", morphName);
                continue;
            }
            morphNameToIdx[charstr] = i;
        }

        for (const vmd::VmdBoneFrame &frame : _vmdLoader.motion.bone_frames) {
            int boneIdx = findOrNegOne(boneNameToPhysicsNodeIdx, frame.name);
            if (boneIdx < 0) {
                continue;
            }
            if (frame.frame > totalFrameCount) {
                totalFrameCount = frame.frame;
            }
            SCNNode *bone = [_geomController boneAtIndex:boneIdx];
            animationData.addBoneFrame(boneIdx, frame, boneInitialBindTransforms[boneIdx]); //bone.simdTransform);
        }
        for (const vmd::VmdFaceFrame &frame : _vmdLoader.motion.face_frames) {
            int morphIdx = findOrNegOne(morphNameToIdx, frame.face_name);
            if (morphIdx < 0) {
                continue;
            }
            if (frame.frame > totalFrameCount) {
                totalFrameCount = frame.frame;
            }
            animationData.addMorphFrame(morphIdx, frame);
        }
        animationData.sortFrames(); // vmd files are not sorted.
    }
    return self;
}

- (void)tick:(NSTimeInterval)time {
    if (_initTime == 0) {
        _initTime = time;
    }
    if (_paused) {
        _initTime += (time - lastFrameTime);
    }
    lastFrameTime = time;
    if (_paused) {
        return;
    }
    [self setCurFrame:(time - _initTime) * 30];
}

- (float)curFrame {
    return curFrame;
}

- (void)setCurFrame:(float)newCurFrame {
    curFrame = newCurFrame;
    if (curFrame > totalFrameCount) {
        curFrame = 0;
        _initTime = lastFrameTime;
    }

    for (size_t boneIdx = 0; boneIdx < animationData.bones.size(); boneIdx++) {
        auto &state = animationData.bones[boneIdx];
        if (!state.updateCurFrameAndCheckActive(curFrame)) {
            continue;
        }
        SCNNode *curNode = [_geomController boneAtIndex:(int)boneIdx];
        auto newBoneState = state.interpolate(curFrame);
        curNode.simdPosition = newBoneState.position;
        curNode.simdOrientation = newBoneState.orientation;
    }
    BOOL upd = (_mmdNode.morpher != nil);
    for (size_t morphIdx = 0; morphIdx < animationData.morphs.size(); morphIdx++) {
        auto &state = animationData.morphs[morphIdx];
        if (!state.updateCurFrameAndCheckActive(curFrame)) {
            continue;
        }
        if (upd) {
            [_mmdNode.morpher setWeight:state.interpolate(curFrame) forTargetAtIndex:morphIdx];
        }
    }
}

@end
