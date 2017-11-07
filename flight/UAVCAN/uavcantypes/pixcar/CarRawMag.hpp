/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/rdu/Workspace/dronin/dRonin/flight/UAVCAN/pixcar/25003.CarRawMag.uavcan
 */

#ifndef PIXCAR_CARRAWMAG_HPP_INCLUDED
#define PIXCAR_CARRAWMAG_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
uint32 time_stamp
float32[3] mag
******************************************************************************/

/********************* DSDL signature source definition ***********************
pixcar.CarRawMag
saturated uint32 time_stamp
saturated float32[3] mag
******************************************************************************/

#undef time_stamp
#undef mag

namespace pixcar
{

template <int _tmpl>
struct UAVCAN_EXPORT CarRawMag_
{
    typedef const CarRawMag_<_tmpl>& ParameterType;
    typedef CarRawMag_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > time_stamp;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > mag;
    };

    enum
    {
        MinBitLen
            = FieldTypes::time_stamp::MinBitLen
            + FieldTypes::mag::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::time_stamp::MaxBitLen
            + FieldTypes::mag::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::time_stamp >::Type time_stamp;
    typename ::uavcan::StorageType< typename FieldTypes::mag >::Type mag;

    CarRawMag_()
        : time_stamp()
        , mag()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<128 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 25003 };

    static const char* getDataTypeFullName()
    {
        return "pixcar.CarRawMag";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool CarRawMag_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        time_stamp == rhs.time_stamp &&
        mag == rhs.mag;
}

template <int _tmpl>
bool CarRawMag_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(time_stamp, rhs.time_stamp) &&
        ::uavcan::areClose(mag, rhs.mag);
}

template <int _tmpl>
int CarRawMag_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::time_stamp::encode(self.time_stamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::mag::encode(self.mag, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int CarRawMag_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::time_stamp::decode(self.time_stamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::mag::decode(self.mag, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature CarRawMag_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xC950CD0CDB7EAD62ULL);

    FieldTypes::time_stamp::extendDataTypeSignature(signature);
    FieldTypes::mag::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef CarRawMag_<0> CarRawMag;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::pixcar::CarRawMag > _uavcan_gdtr_registrator_CarRawMag;

}

} // Namespace pixcar

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::pixcar::CarRawMag >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::pixcar::CarRawMag::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::pixcar::CarRawMag >::stream(Stream& s, ::pixcar::CarRawMag::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "time_stamp: ";
    YamlStreamer< ::pixcar::CarRawMag::FieldTypes::time_stamp >::stream(s, obj.time_stamp, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "mag: ";
    YamlStreamer< ::pixcar::CarRawMag::FieldTypes::mag >::stream(s, obj.mag, level + 1);
}

}

namespace pixcar
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::pixcar::CarRawMag::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::pixcar::CarRawMag >::stream(s, obj, 0);
    return s;
}

} // Namespace pixcar

#endif // PIXCAR_CARRAWMAG_HPP_INCLUDED