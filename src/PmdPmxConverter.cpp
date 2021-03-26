/*
pmxar (PmdPmxConverter.cpp)
Copyright (c) 2018 Lyuma

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "PmdPmxConverter.h"

#include <map>
#include <math.h>

extern std::string convertSjisToUtf8(const std::string &in);

void convertVertex(size_t i, const pmd::PmdVertex &in, pmx::PmxVertex &out) {
    out.edge = in.edge_invisible ? 0 : 1;
    if (in.bone_index[1] < 0 || in.bone_index[1] == 65535 || in.bone_index[1] ==    in.bone_index[0] || in.bone_weight >= 100) {
        out.skinning_type = pmx::PmxVertexSkinningType::BDEF1;
        auto bdef1 = new pmx::PmxVertexSkinningBDEF1;
        bdef1->bone_index = in.bone_index[0];
        out.skinning.reset(bdef1);
    } else {
        out.skinning_type = pmx::PmxVertexSkinningType::BDEF2;
        auto bdef2 = new pmx::PmxVertexSkinningBDEF2;
        bdef2->bone_index1 = in.bone_index[0];
        bdef2->bone_index2 = in.bone_index[1];
        bdef2->bone_weight = (float)in.bone_weight / 100.f;
        out.skinning.reset(bdef2);
    }
    memcpy(out.position, in.position, 3 * sizeof(float));
    memcpy(out.normal, in.normal, 3 * sizeof(float));
    memcpy(out.uv, in.uv, 2 * sizeof(float));
}

void convertMaterial(size_t i, const pmd::PmdMaterial &in, pmx::PmxMaterial &out, const std::map<std::string, size_t> &texNames) {
    memcpy(out.ambient, in.ambient, sizeof(float) * 3);
    memcpy(out.diffuse, in.diffuse, sizeof(float) * 4);
    memcpy(out.specular, in.specular, sizeof(float) * 3);
    out.edge_size = in.edge_flag ? 1. : 0.;
    out.edge_color[3] = in.edge_flag ? 1. : 0.;
    out.index_count = in.index_count;
    out.specularlity = in.power;
    auto it = texNames.find(in.sphere_filename);
    if (it == texNames.end()) {
        out.sphere_texture_index = -1;
    } else {
        out.sphere_texture_index = (int) it->second;
    }
    it = texNames.find(in.texture_filename);
    if (it == texNames.end()) {
        out.diffuse_texture_index = -1;
    } else {
        out.diffuse_texture_index = (int)it->second;
    }
    out.common_toon_flag = true;
    out.toon_texture_index = in.toon_index;
}

void convertBone(size_t, const pmd::PmdBone &in, pmx::PmxBone &out) {
    out.bone_flag = 0x19 | 0x6; // indexed tail, visible, enabled, rotatible, movable
    switch (in.bone_type) {
    case pmd::BoneType::Rotation:
        out.bone_flag &= ~0x4;
        break;
    case pmd::BoneType::RotationAndMove:
        break;
    case pmd::BoneType::RotationEffectable:
        out.bone_flag |= 0x100;
        out.grant_parent_index = in.ik_parent_bone_index;
        out.grant_weight = 1.0;
        break;
    case pmd::BoneType::IkTarget:
        break;
    case pmd::BoneType::Invisible:
        out.bone_flag &= (~0x8);
        break;
    case pmd::BoneType::RotationMovement:
        out.bone_flag |= 0x300;
        out.grant_parent_index = in.tail_pos_bone_index;
        out.grant_weight = (float)in.ik_parent_bone_index / 100.f;
        break;
    case pmd::BoneType::Twist:
    case pmd::BoneType::IkEffector:
    case pmd::BoneType::Unknown:
        break;
    case pmd::BoneType::IkEffectable:
    default:
        out.bone_flag &= ~0x4;
        break;
    }
    memcpy(out.position, in.bone_head_pos, sizeof(float) * 3);
    if (in.parent_bone_index == 65535) {
        out.parent_index = -1;
    } else {
        out.parent_index = in.parent_bone_index;
    }
    out.bone_name = convertSjisToUtf8(in.name);
    out.bone_english_name = in.name_english;
    out.target_index = in.tail_pos_bone_index;
    out.ik_target_bone_index = -1;
}

void convertIK(const std::vector<pmd::PmdBone> &inBones, const std::vector<pmd::PmdIk> &inIK, pmx::PmxBone *outBones) {
/*
    for (size_t i = 0; i < inBones.size(); i++) {
        if (inBones[i].bone_type == pmd::BoneType::IkTarget) {
            int ik_parent = inBones[i].ik_parent_bone_index;
            if (ik_parent < 0 || ik_parent >= inBones.size()) {
                NSLog(@"Error parsing IK %d %d", i, ik_parent);
                return; // invalid data.
            }
            outBones[ik_parent].ik_target_bone_index = (int)i;
        }
    }
*/
    for (const pmd::PmdIk &ik : inIK) {
        int boneidx = ik.ik_bone_index;
        if (boneidx < 0 || boneidx >= inBones.size()) {
            continue;
        }
        pmx::PmxBone &outBone = outBones[ik.ik_bone_index];
        outBone.bone_flag |= (1<<5);
        outBone.ik_loop = ik.interations;
        outBone.ik_loop_angle_limit = ik.angle_limit;
        outBone.ik_target_bone_index = ik.target_bone_index;
        outBone.ik_links.reset(new pmx::PmxIkLink[outBone.ik_link_count = (int)ik.ik_child_bone_index.size()]);
        for (int i = 0; i < outBone.ik_link_count; i++) {
            if (ik.ik_child_bone_index[i] >= inBones.size()) {
                continue;
            }
            bool isKnee = outBones[ik.ik_child_bone_index[i]].bone_name.find("ひざ") != std::string::npos;
            // TODO: If "ひざ" in child_bone
            auto &outLink = outBone.ik_links[i];
            outLink.link_target = ik.ik_child_bone_index[i];
            if (isKnee) {
                outLink.max_radian[0] = -0.5 * M_PI / 180.f;
                outLink.max_radian[1] = 0;
                outLink.max_radian[2] = 0;
                outLink.min_radian[0] = -M_PI/2.f;
                outLink.min_radian[1] = 0;
                outLink.min_radian[2] = 0;
            }
        }
    }
}

void convertMorph(const pmd::PmdFace &in, pmx::PmxMorph &out) {
    out.morph_name =  convertSjisToUtf8(in.name);
    out.morph_english_name = in.name_english;
    switch (in.type) {
    case pmd::FaceCategory::Base:
        out.category = pmx::MorphCategory::ReservedCategory;
        break;
    case pmd::FaceCategory::Eye:
        out.category = pmx::MorphCategory::Eye;
        break;
    case pmd::FaceCategory::Eyebrow:
        out.category = pmx::MorphCategory::Eyebrow;
        break;
    case pmd::FaceCategory::Mouth:
        out.category = pmx::MorphCategory::Mouth;
        break;
    case pmd::FaceCategory::Other:
    default:
        out.category = pmx::MorphCategory::Other;
        break;
    }
    out.morph_type = pmx::MorphType::Vertex;
    out.vertex_offsets.reset(new pmx::PmxMorphVertexOffset[out.offset_count = (int)in.vertices.size()]);
    for (size_t i = 0; i < in.vertices.size(); i++) {
        out.vertex_offsets[i].vertex_index = in.vertices[i].vertex_index;
        memcpy(out.vertex_offsets[i].position_offset, in.vertices[i].position, sizeof(float) * 3);
    }
}

void convertRigidBody(const pmd::PmdRigidBody &in, float(&inBonePos)[3], pmx::PmxRigidBody &out) {
    out.rotation_attenuation = in.anglar_damping;
    out.move_attenuation = in.linear_damping;
    out.group = in.group_index;
    out.mask = in.mask;
    out.girid_body_name =  convertSjisToUtf8(in.name);
    out.girid_body_english_name = in.name;
    memcpy(out.orientation, in.orientation, sizeof(float) * 3);
    out.position[0] = inBonePos[0] + in.position[0];
    out.position[1] = inBonePos[1] + in.position[1];
    out.position[2] = inBonePos[2] + in.position[2];
    if (in.related_bone_index < 0 || (size_t)in.related_bone_index >= 65535) {
        out.target_bone = -1;
    } else {
        out.target_bone = in.related_bone_index;
    }
    out.friction = in.friction;
    out.repulsion = in.restitution; //?????????
    switch (in.rigid_type) {
    case pmd::RigidBodyType::BoneConnected:
        out.physics_calc_type = 0;
        break;
    case pmd::RigidBodyType::ConnectedPhysics:
        out.physics_calc_type = 2;
        break;
    case pmd::RigidBodyType::Physics:
    default:
        out.physics_calc_type = 1;
        break;
    }
    switch (in.shape) {
    case pmd::RigidBodyShape::Box:
        out.shape = 1;
        break;
    case pmd::RigidBodyShape::Cpusel:
        out.shape = 2;
        break;
    case pmd::RigidBodyShape::Sphere:
    default:
        out.shape = 0;
        break;
    }
    memcpy(out.size, in.size, sizeof(float) * 3);
    out.mass = in.weight; // correct?
}

void convertJoint(const pmd::PmdConstraint &in, pmx::PmxJoint &out) {
    out.joint_type = pmx::PmxJointType::Generic6DofSpring;
    memcpy(out.param.rotation_limitation_min, in.angular_lower_limit, sizeof(float) * 3);
    memcpy(out.param.rotation_limitation_max, in.angular_upper_limit, sizeof(float) * 3);
    memcpy(out.param.move_limitation_min, in.linear_lower_limit, sizeof(float) * 3);
    memcpy(out.param.move_limitation_max, in.linear_upper_limit, sizeof(float) * 3);
    memcpy(out.param.spring_rotation_coefficient, in.angular_stiffness, sizeof(float) * 3);
    memcpy(out.param.spring_move_coefficient, in.linear_stiffness, sizeof(float) * 3);
    out.joint_name =  convertSjisToUtf8(in.name);
    out.joint_english_name = in.name;
    memcpy(out.param.orientaiton, in.orientation, sizeof(float) * 3);
    memcpy(out.param.position, in.position, sizeof(float) * 3);
    out.param.rigid_body1 = in.rigid_body_index_a;
    out.param.rigid_body2 = in.rigid_body_index_b;
}

bool convertPmdToPmx(const pmd::PmdModel &inModel, pmx::PmxModel &outModel) {
    outModel.model_name =  convertSjisToUtf8(inModel.header.name);
    outModel.model_english_name = inModel.header.name_english;
    outModel.model_comment =  convertSjisToUtf8(inModel.header.comment);
    outModel.model_english_comment = inModel.header.comment_english;
    outModel.version = inModel.version;
    
    outModel.Init();
    outModel.vertices.reset(new pmx::PmxVertex[outModel.vertex_count = (int)inModel.vertices.size()]);
    outModel.indices.reset(new int[outModel.index_count = (int)inModel.indices.size()]);
    outModel.materials.reset(new pmx::PmxMaterial[outModel.material_count = (int)inModel.materials.size()]);
    outModel.bones.reset(new pmx::PmxBone[outModel.bone_count = (int)inModel.bones.size()]);
    outModel.morphs.reset(new pmx::PmxMorph[outModel.morph_count = (int)inModel.faces.size()]);
    outModel.rigid_bodies.reset(new pmx::PmxRigidBody[outModel.rigid_body_count = (int)inModel.rigid_bodies.size()]);
    outModel.joints.reset(new pmx::PmxJoint[outModel.joint_count = (int)inModel.constraints.size()]);
    
    for (size_t i = 0; i < inModel.vertices.size(); i++) {
        convertVertex(i, inModel.vertices[i], outModel.vertices[i]);
    }
    std::copy(inModel.indices.begin(), inModel.indices.end(), outModel.indices.get());
    std::map<std::string, size_t> texNames;
    for (size_t i = 0; i < inModel.materials.size(); i++) {
        const auto &mat = inModel.materials[i];
        if (!mat.texture_filename.empty()) {
            texNames.insert(std::make_pair(mat.texture_filename, 0));
        }
        if (!mat.sphere_filename.empty()) {
            texNames.insert(std::make_pair(mat.sphere_filename, 0));
        }
    }
    outModel.textures.reset(new std::string[ outModel.texture_count = (int)texNames.size()]);
    size_t i = 0;
    for (auto &pair : texNames) {
        pair.second = i;
        outModel.textures[i] =  convertSjisToUtf8(pair.first);
        i++;
    }
    for (size_t i = 0; i < inModel.materials.size(); i++) {
        convertMaterial(i, inModel.materials[i], outModel.materials[i], texNames);
    }
    for (size_t i = 0; i < inModel.bones.size(); i++) {
        convertBone(i, inModel.bones[i], outModel.bones[i]);
    }
    convertIK(inModel.bones, inModel.iks, outModel.bones.get());
    for (size_t i = 0; i < inModel.faces.size(); i++) {
        convertMorph(inModel.faces[i], outModel.morphs[i]);
    }
    for (size_t i = 0; i < inModel.rigid_bodies.size(); i++) {
        float bonePos[3] = {0, 0, 0};
        if ((size_t)inModel.rigid_bodies[i].related_bone_index < inModel.bones.size()) {
            memcpy(bonePos, inModel.bones[inModel.rigid_bodies[i].related_bone_index].bone_head_pos, sizeof(float) * 3);
        }
        convertRigidBody(inModel.rigid_bodies[i], bonePos, outModel.rigid_bodies[i]);
    }
    for (size_t i = 0; i < inModel.constraints.size(); i++) {
        convertJoint(inModel.constraints[i], outModel.joints[i]);
    }
    return true;
}
