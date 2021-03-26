#include "MMDPmxParser.h"
#include "MMDPmdParser.h"
#include "PmdPmxConverter.h"
#include <fstream>
#include "shiftjis.h"

// From https://stackoverflow.com/a/33170901
std::string convertSjisToUtf8(const std::string &input)
{
    std::string output(3 * input.length(), ' '); //ShiftJis won't give 4byte UTF8, so max. 3 byte per input char are needed
    size_t indexInput = 0, indexOutput = 0;

    while(indexInput < input.length())
    {
        char arraySection = ((uint8_t)input[indexInput]) >> 4;

        size_t arrayOffset;
        if(arraySection == 0x8) arrayOffset = 0x100; //these are two-byte shiftjis
        else if(arraySection == 0x9) arrayOffset = 0x1100;
        else if(arraySection == 0xE) arrayOffset = 0x2100;
        else arrayOffset = 0; //this is one byte shiftjis

        //determining real array offset
        if(arrayOffset)
        {
            arrayOffset += (((uint8_t)input[indexInput]) & 0xf) << 8;
            indexInput++;
            if(indexInput >= input.length()) break;
        }
        arrayOffset += (uint8_t)input[indexInput++];
        arrayOffset <<= 1;

        //unicode number is...
        uint16_t unicodeValue = (shiftJIS_convTable[arrayOffset] << 8) | shiftJIS_convTable[arrayOffset + 1];

        //converting to UTF8
        if(unicodeValue < 0x80)
        {
            output[indexOutput++] = unicodeValue;
        }
        else if(unicodeValue < 0x800)
        {
            output[indexOutput++] = 0xC0 | (unicodeValue >> 6);
            output[indexOutput++] = 0x80 | (unicodeValue & 0x3f);
        }
        else
        {
            output[indexOutput++] = 0xE0 | (unicodeValue >> 12);
            output[indexOutput++] = 0x80 | ((unicodeValue & 0xfff) >> 6);
            output[indexOutput++] = 0x80 | (unicodeValue & 0x3f);
        }
    }

    output.resize(indexOutput); //remove the unnecessary bytes
    return output;
}


bool process(const char *filename) {
    pmx::PmxModel model;
    model.Init();
    std::ifstream ifs (filename);
    if (!model.Read(&ifs) || model.bone_count < 1) {
        ifs.seekg(0, std::ios_base::beg);
        std::unique_ptr<pmd::PmdModel> pmdModel (
                    pmd::PmdModel::LoadFromStream(&ifs));
        if (!pmdModel.get()) {
            printf("Failed to parse PMD or PMX %s\n", filename);
            return false;
        }
        if (!convertPmdToPmx(*pmdModel, model)) {
            printf("Failed to convert PMD %s\n", filename);
            return false;
        }
    }
    std::vector<std::string> images;
    for (int i = 0; i < model.texture_count; i++) {
        std::string texStr = model.textures[i];
        std::replace(texStr.begin(), texStr.end(), '\\', '/');
        //if (texStr.find("..") != -1) { // avoid going out of root dir.
        //    texStr.erase(std::remove(texStr.begin(), texStr.end(), '/'), texStr.end());
        //}
        images.push_back(texStr);
    }
    for (int i = 0; i < model.material_count; i++) {
        const pmx::PmxMaterial &pmxmat = model.materials[i];
        printf("Material %02d: %s\n", i, pmxmat.material_name.c_str());
        printf("    English: %s\n", pmxmat.material_english_name.c_str());
        if (pmxmat.diffuse_texture_index >= 0 && pmxmat.diffuse_texture_index < model.texture_count) {
            std::string fn = images[pmxmat.diffuse_texture_index];
            printf("    %s tex: %s\n", fn.find(".sph") != std::string::npos ? "sphere mul" : (fn.find(".spa") != std::string::npos ? "sphere add" : (fn.find("_s") != std::string::npos ? "sphere?" : (fn.find("_n") != std::string::npos ? "normal?" : "diffuse"))), fn.c_str());
        }
        if (pmxmat.sphere_texture_index >= 0 && pmxmat.sphere_texture_index < model.texture_count) {
            printf("    %s tex: %s\n", (pmxmat.sphere_op_mode == 0 ? "unknown sphere" : (pmxmat.sphere_op_mode == 1 ? "sphere mul" : (pmxmat.sphere_op_mode == 2 ? "sphere add" : (pmxmat.sphere_op_mode == 3 ? "detail" : "unknown")))), images[pmxmat.sphere_texture_index].c_str());
        }
        printf("    diffuse col: (%d,%d,%d,%d)\n", (int)(255 * pmxmat.diffuse[0]), (int)(255 * pmxmat.diffuse[1]), (int)(255 * pmxmat.diffuse[2]), (int)(255 * pmxmat.diffuse[3]));
        printf("    specular col: (%d,%d,%d)\n", (int)(255 * pmxmat.specular[0]), (int)(255 * pmxmat.specular[1]), (int)(255 * pmxmat.specular[2]));
        if (pmxmat.specular[0] > 0 && pmxmat.specular[1] > 0 && pmxmat.specular[2] > 0) {
            printf("    specular strength: %g\n", pmxmat.specularlity);
        }
        printf("    ambient col: (%d,%d,%d)\n", (int)(255 * pmxmat.ambient[0]), (int)(255 * pmxmat.ambient[1]), (int)(255 * pmxmat.ambient[2]));
        if (pmxmat.common_toon_flag == 1 && pmxmat.toon_texture_index >= 1 && pmxmat.toon_texture_index <= 10) {
            printf("    toon ramp: toon%02d.bmp (builtin)\n", pmxmat.toon_texture_index);
        } else if (pmxmat.toon_texture_index >= 0 && pmxmat.toon_texture_index < model.texture_count  && pmxmat.common_toon_flag == 0) {
            printf("    toon ramp: %s\n", images[pmxmat.toon_texture_index].c_str());
        }
        printf("    outlines: %s\n", (pmxmat.flag & (1 << 4)) ? "on" : "off");
        // flag bit 4 relates to edge drawing
        // edge_color / edge_size
		if (pmxmat.flag & (1 << 4)) {
            printf("    edge col: (%d,%d,%d,%d)\n", (int)(255 * pmxmat.edge_color[0]), (int)(255 * pmxmat.edge_color[1]), (int)(255 * pmxmat.edge_color[2]), (int)(255*pmxmat.edge_color[3]));
            printf("    edge size: %g\n", pmxmat.edge_size);
        }
        printf("    culling: %s\n", (pmxmat.flag & (1 << 0)) ? "off" : "back");
        if (pmxmat.flag & (1 << 7)) {
            printf("    wireframe mode: %s\n", (pmxmat.flag & (1 << 7)) ? "on" : "off");
        }
        // flag bits 1,2,3 relate to shadow
        if (pmxmat.flag & ((1 << 1)|(1 << 2)|(1 << 3))) {
            printf("    cast env shadow: %s\n", (pmxmat.flag & (1 << 1)) ? "on" : "off");
            printf("    cast shadows: %s\n", (pmxmat.flag & (1 << 2)) ? "on" : "off");
            printf("    receive shadows: %s\n", (pmxmat.flag & (1 << 3)) ? "on" : "off");
        }
        if (pmxmat.flag & (1 << 5)) {
            printf("    use vertex colors: %s\n", (pmxmat.flag & (1 << 5)) ? "on" : "off");
        }
        if (pmxmat.flag & (1 << 6)) {
            printf("    points: %d\n", pmxmat.index_count);
        } else {
            printf("    triangles: %d\n", pmxmat.index_count / 3);
        }
        if (pmxmat.memo.size() > 0) {
            printf("    metadata:\n%s\n", pmxmat.memo.c_str());
        }
        printf("\n");
    }
    printf("All Done with %s\n", filename);
    return true;
}

int main(int argc, const char**argv) {
    for (int i = 1; i < argc; i++) {
        process(argv[i]);
    }
}

