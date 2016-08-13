/****************************************************************************/
/// @file    MFXImageHelper.cpp
/// @author  Daniel Krajzewicz
/// @date    2005-05-04
/// @version $Id: MFXImageHelper.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// missing_desc
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2005-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <fx.h>
#include <FXPNGImage.h>
#include <FXJPGImage.h>
#include <FXTIFImage.h>
#include <utils/common/ToString.h>
#include "MFXImageHelper.h"

#include <cassert>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

void
MFXImageHelper::checkSupported(FXString ext) {
    if (comparecase(ext, "png") == 0) {
        if (!FXPNGImage::supported) {
            throw InvalidArgument("Fox was compiled without png support!");
        }
    } else if (comparecase(ext, "jpg") == 0 || comparecase(ext, "jpeg") == 0) {
        if (!FXJPGImage::supported) {
            throw InvalidArgument("Fox was compiled without jpg support!");
        }
    } else if (comparecase(ext, "tif") == 0 || comparecase(ext, "tiff") == 0) {
        if (!FXTIFImage::supported) {
            throw InvalidArgument("Fox was compiled without tif support!");
        }
    }
}


FXImage*
MFXImageHelper::loadImage(FXApp* a, const std::string& file) {
    FXString ext = FXPath::extension(file.c_str());
    checkSupported(ext);
    FXImage* img = NULL;
    if (comparecase(ext, "gif") == 0) {
        img = new FXGIFImage(a, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else if (comparecase(ext, "bmp") == 0) {
        img = new FXBMPImage(a, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else if (comparecase(ext, "xpm") == 0) {
        img = new FXXPMImage(a, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else if (comparecase(ext, "pcx") == 0) {
        img = new FXPCXImage(a, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else if (comparecase(ext, "ico") == 0 || comparecase(ext, "cur") == 0) {
        img = new FXICOImage(a, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else if (comparecase(ext, "tga") == 0) {
        img = new FXTGAImage(a, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else if (comparecase(ext, "rgb") == 0) {
        img = new FXRGBImage(a, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else if (comparecase(ext, "xbm") == 0) {
        img = new FXXBMImage(a, NULL, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else if (comparecase(ext, "png") == 0) {
        img = new FXPNGImage(a, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else if (comparecase(ext, "jpg") == 0 || comparecase(ext, "jpeg") == 0) {
        img = new FXJPGImage(a, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else if (comparecase(ext, "tif") == 0 || comparecase(ext, "tiff") == 0) {
        img = new FXTIFImage(a, NULL, IMAGE_KEEP | IMAGE_SHMI | IMAGE_SHMP);
    } else {
        throw InvalidArgument("Unknown file extension '" + toString(ext.text()) + "' for image '" + file + "'!");
    }

    FXFileStream stream;
    if (img != NULL && stream.open(file.c_str(), FXStreamLoad)) {
        a->beginWaitCursor();
        img->loadPixels(stream);
        stream.close();

        img->create();
        a->endWaitCursor();
    } else {
        delete img;
        throw InvalidArgument("Loading failed!");
    }
    return img;
}


FXbool
MFXImageHelper::scalePower2(FXImage* image, const int maxSize) {
    FXint newHeight = 0;
    for (FXint exp = 30; exp >= 0; exp--) {
        newHeight = 2 << exp;
        if (newHeight <= maxSize && (image->getHeight() & newHeight)) {
            break;
        }
    }
    if (2 * newHeight <= maxSize && (2 * newHeight - image->getHeight() < image->getHeight() - newHeight)) {
        newHeight *= 2;
    }
    FXint newWidth = 0;
    for (FXint exp = 30; exp >= 0; exp--) {
        newWidth = 2 << exp;
        if (newWidth <= maxSize && (image->getWidth() & newWidth)) {
            break;
        }
    }
    if (2 * newWidth <= maxSize && (2 * newWidth - image->getWidth() < image->getWidth() - newWidth)) {
        newWidth *= 2;
    }
    if (newHeight == image->getHeight() && newWidth == image->getWidth()) {
        return false;
    }
    image->scale(newWidth, newHeight);
    return true;
}


// smell: yellow (the save functions may have additional options, not regarded)
// Save file
FXbool
MFXImageHelper::saveImage(const std::string& file,
                          int width, int height, FXColor* data) {
    FXString ext = FXPath::extension(file.c_str());
    checkSupported(ext);
    FXFileStream stream;
    if (!stream.open(file.c_str(), FXStreamSave)) {
        throw InvalidArgument("Could not open file for writing!");
    }
    if (comparecase(ext, "gif") == 0) {
        return fxsaveGIF(stream, data, width, height, false /* !!! "fast" */);
    } else if (comparecase(ext, "bmp") == 0) {
        return fxsaveBMP(stream, data, width, height);
    } else if (comparecase(ext, "xpm") == 0) {
        return fxsaveXPM(stream, data, width, height);
    } else if (comparecase(ext, "pcx") == 0) {
        return fxsavePCX(stream, data, width, height);
    } else if (comparecase(ext, "ico") == 0 || comparecase(ext, "cur") == 0) {
        return fxsaveICO(stream, data, width, height);
    } else if (comparecase(ext, "tga") == 0) {
        return fxsaveTGA(stream, data, width, height);
    } else if (comparecase(ext, "rgb") == 0) {
        return fxsaveRGB(stream, data, width, height);
    } else if (comparecase(ext, "xbm") == 0) {
        return fxsaveXBM(stream, data, width, height);
    } else if (comparecase(ext, "png") == 0) {
        return fxsavePNG(stream, data, width, height);
    } else if (comparecase(ext, "jpg") == 0 || comparecase(ext, "jpeg") == 0) {
        return fxsaveJPG(stream, data, width, height, 75);
    } else if (comparecase(ext, "tif") == 0 || comparecase(ext, "tiff") == 0) {
        return fxsaveTIF(stream, data, width, height, 0);
    }
    throw InvalidArgument("Unknown file extension for image!");
}



/****************************************************************************/

