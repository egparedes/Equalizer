
/* Copyright (c) 2007-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *               2015, David Steiner <steiner@ifi.uzh.ch> 
 *               2015, Enrique G. Paredes <egparedes@ifi.uzh.ch> 
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Eyescale Software GmbH nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "localInitData.h"
#include "frameData.h"

#pragma warning( disable: 4275 )
#include <boost/program_options.hpp>
#pragma warning( default: 4275 )

#include <cctype>

#ifndef MIN
#  define MIN LB_MIN
#endif

namespace po = boost::program_options;

namespace eqPly
{
LocalInitData::LocalInitData()
    : _pathInFilename("")
    , _pathOutFilename("")
    , _maxFrames( 0xffffffffu )
    , _color( true )
    , _isResident( false )
    , _treePartition("")
    , _createLongShowcase( false )
    , _showcaseDollyArgs( .0, .0, .0 )
    , _showcaseRadiusArgs( .0, .0, .0 )
    , _showcaseAngleArgs( .0, .0, .0 )
{
    _filenames.push_back( lunchbox::getRootPath() +
                          "/share/Equalizer/data" );
}

LocalInitData& LocalInitData::operator = ( const LocalInitData& from )
{
    _maxFrames         = from._maxFrames;
    _exitAfterPlayback = from._exitAfterPlayback;
    _color             = from._color;
    _isResident        = from._isResident;
    _filenames         = from._filenames;
    _pathInFilename    = from._pathInFilename;
    _pathOutFilename   = from._pathOutFilename;
    _treePartition     = from._treePartition;
    _createLongShowcase   = from._createLongShowcase;
    _showcaseDollyArgs    = from._showcaseDollyArgs;
    _showcaseRadiusArgs   = from._showcaseRadiusArgs;
    _showcaseAngleArgs    = from._showcaseAngleArgs;

    setWindowSystem( from.getWindowSystem( ));
    setRenderMode( from.getRenderMode( ));
    if( from.useGLSL( ))
        enableGLSL();
    if( from.useInvertedFaces( ))
        enableInvertedFaces();
    if( !from.showLogo( ))
        disableLogo();
    if( !from.useROI( ))
        disableROI();
    if( from.useOutOfCore() )
        enableOutOfCore();

    return *this;
}

void LocalInitData::parseArguments( const int argc, char** argv )
{
        std::string wsHelp = "Window System API ( one of: ";
#ifdef AGL
        wsHelp += "AGL ";
#endif
#ifdef GLX
    wsHelp += "GLX ";
#endif
#ifdef WGL
        wsHelp += "WGL ";
#endif
#ifdef EQUALIZER_USE_QT5WIDGETS
    wsHelp += "Qt ";
#endif
        wsHelp += ")";

    bool showHelp( false );
    std::vector<std::string> userDefinedModelPath;
    bool userDefinedBlackWhiteMode( false );
    std::string userDefinedWindowSystem("");
    std::string userDefinedRenderMode("");
    std::string showcaseArgs("");
    bool userDefinedUseGLSL( false );
    bool userDefinedInvertFaces( false );
    bool userDefinedDisableLogo( false );
    bool userDefinedDisableROI( false );
    bool userDefinedOutOfCore( false );

    const std::string& desc = EqPly::getHelp();
    po::options_description options( desc + " Version " +
                                     eq::Version::getString( ));
    options.add_options()
        ( "help,h", po::bool_switch(&showHelp)->default_value( false ),
          "produce help message" )
        ( "model,m",
          po::value<std::vector<std::string> >( &userDefinedModelPath ),
          "ply model file names or directories" )
        ( "blackAndWhite,b",
          po::bool_switch(&userDefinedBlackWhiteMode)->default_value( false ),
          "Don't use colors from ply file" )
        ( "resident,r", po::bool_switch(&_isResident)->default_value( false ),
          "Keep client resident (see resident mode documentation on website)" )
        ( "numFrames,n",
          po::value<uint32_t>(&_maxFrames)->default_value(0xffffffffu),
          "Maximum number of rendered frames")
        ( "windowSystem,w", po::value<std::string>( &userDefinedWindowSystem ),
          wsHelp.c_str() )
        ( "renderMode,c", po::value<std::string>( &userDefinedRenderMode ),
          "Rendering Mode (immediate|displayList|VBO)" )
        ( "glsl,g",
          po::bool_switch(&userDefinedUseGLSL)->default_value( false ),
          "Enable GLSL shaders" )
        ( "invertFaces,i"
          , po::bool_switch(&userDefinedInvertFaces)->default_value( false ),
          "Invert faces (valid during binary file creation)" )
        ( "cameraPath,a", po::value<std::string>(&_pathInFilename),
          "File containing camera path animation" )
        ( "exit,e", po::bool_switch(&_exitAfterPlayback)->default_value( false ),
          "Exit after playing camera path animation" )
        ( "cameraPathOut,t", po::value<std::string>(&_pathOutFilename),
          "File to save camera path animation to" )
        ( "noOverlay,o",
          po::bool_switch(&userDefinedDisableLogo)->default_value( false ),
          "Disable overlay logo" )
        ( "disableROI,d",
          po::bool_switch(&userDefinedDisableROI)->default_value( false ),
          "Disable region of interest (ROI)" )
        ( "treePartition,p",
          po::value<std::string>(&_treePartition)->default_value( "kd" ),
          "Tree partition strategy (kd|z)")
        ( "longShowcase,l", po::value<std::string>( &showcaseArgs )
                                ->implicit_value("-0.1:0.0005:1.1/0.55:0.0001:0.7/0.0:0.0005:1.0"),
          "Create path in the model longest axis with the parameters dolly/radius/angle "
          "(relative to bounding box size, see default)" )
        ( "outOfCore,z"
          , po::bool_switch(&userDefinedOutOfCore)->default_value( false ),
          "Use out-of-core model rendering (only works with binary files)" );

    po::variables_map variableMap;

    try
        {
        // parse program options, ignore all non related options
        po::store( po::command_line_parser( argc, argv ).options(
                       options ).allow_unregistered().run(),
                   variableMap );
        po::notify( variableMap );
        }
    catch( std::exception& exception )
        {
        LBERROR << "Error parsing command line: " << exception.what()
                << std::endl;
        eq::exit(); // cppcheck-suppress unreachableCode
        ::exit( EXIT_FAILURE );
    }


    // Evaluate parsed command line options
    if( showHelp )
    {
        std::cout << options << std::endl;
        eq::exit(); // cppcheck-suppress unreachableCode
        ::exit( EXIT_SUCCESS );
    }

    if( variableMap.count("model") > 0 )
    {
        _filenames.clear();
        _filenames = userDefinedModelPath;
    }

    _color = !userDefinedBlackWhiteMode;

    if( variableMap.count("windowSystem") > 0 )
        setWindowSystem( userDefinedWindowSystem );

    if( variableMap.count("renderMode") > 0 )
    {
        std::transform( userDefinedRenderMode.begin(),
                        userDefinedRenderMode.end(),
                        userDefinedRenderMode.begin(),
                        (int(*)(int))std::tolower );

        if( userDefinedRenderMode == "immediate" )
            setRenderMode( triply::RENDER_MODE_IMMEDIATE );
        else if( userDefinedRenderMode == "displaylist" )
            setRenderMode( triply::RENDER_MODE_DISPLAY_LIST );
        else if( userDefinedRenderMode == "vbo" )
            setRenderMode( triply::RENDER_MODE_BUFFER_OBJECT );
    }

    if( userDefinedUseGLSL )
            enableGLSL();

    if( userDefinedInvertFaces)
            enableInvertedFaces();

    if( userDefinedDisableLogo )
            disableLogo();

    if( userDefinedDisableROI )
            disableROI();

    if( variableMap.count("longShowcase") > 0 )
    {
        std::istringstream iss(showcaseArgs);

        bool isOk = true;
        char separator;
        for( unsigned i=0; i < 3 && isOk; ++i )
        {
            iss >> _showcaseDollyArgs[i];
            iss >> separator;
            isOk = isOk && (separator == ':' || separator == '/');
        }
        for( unsigned i=0; i < 3 && isOk; ++i )
        {
            iss >> _showcaseRadiusArgs[i];
            iss >> separator;
            isOk = isOk && (separator == ':' || separator == '/');
        }
        iss >> separator;
        for( unsigned i=0; i < 3 && isOk; ++i )
        {
            iss >> _showcaseAngleArgs[i];
            iss >> separator;
            isOk = isOk && (separator == ':' || separator == '/');
        }

        if( !isOk )
        {
            _showcaseDollyArgs = eq::Vector3f( -0.1f, 0.0005f, 1.0f );
            _showcaseRadiusArgs = eq::Vector3f( 0.55f, 0.0001f, 0.7f );
            _showcaseAngleArgs = eq::Vector3f( 0.0f, 0.0005f, 1.0f );
        }

        _createLongShowcase = true;
    }

    if (userDefinedOutOfCore)
        enableOutOfCore();
}

}
