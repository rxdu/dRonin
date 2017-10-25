/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/rdu/Workspace/dronin/dRonin/flight/UAVCAN/dsdl/uavcan/equipment/air_data/1028.StaticPressure.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Static pressure.
#

float32 static_pressure                 # Pascal
float16 static_pressure_variance        # Pascal^2
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.air_data.StaticPressure
saturated float32 static_pressure
saturated float16 static_pressure_variance
******************************************************************************/

#undef static_pressure
#undef static_pressure_variance

namespace uavcan
{
namespace equipment
{
namespace air_data
{

template <int _tmpl>
struct UAVCAN_EXPORT StaticPressure_
{
    typedef const StaticPressure_<_tmpl>& ParameterType;
    typedef StaticPressure_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > static_pressure;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > static_pressure_variance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::static_pressure::MinBitLen
            + FieldTypes::static_pressure_variance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::static_pressure::MaxBitLen
            + FieldTypes::static_pressure_variance::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::static_pressure >::Type static_pressure;
    typename ::uavcan::StorageType< typename FieldTypes::static_pressure_variance >::Type static_pressure_variance;

    StaticPressure_()
        : static_pressure()
        , static_pressure_variance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<48 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1028 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.air_data.StaticPressure";
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
bool StaticPressure_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        static_pressure == rhs.static_pressure &&
        static_pressure_variance == rhs.static_pressure_variance;
}

template <int _tmpl>
bool StaticPressure_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(static_pressure, rhs.static_pressure) &&
        ::uavcan::areClose(static_pressure_variance, rhs.static_pressure_variance);
}

template <int _tmpl>
int StaticPressure_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::static_pressure::encode(self.static_pressure, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::static_pressure_variance::encode(self.static_pressure_variance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int StaticPressure_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::static_pressure::decode(self.static_pressure, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::static_pressure_variance::decode(self.static_pressure_variance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature StaticPressure_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xCDC7C43412BDC89AULL);

    FieldTypes::static_pressure::extendDataTypeSignature(signature);
    FieldTypes::static_pressure_variance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef StaticPressure_<0> StaticPressure;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::air_data::StaticPressure > _uavcan_gdtr_registrator_StaticPressure;

}

} // Namespace air_data
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::air_data::StaticPressure >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::air_data::StaticPressure::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::air_data::StaticPressure >::stream(Stream& s, ::uavcan::equipment::air_data::StaticPressure::ParameterType obj, const int level)
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
    s << "static_pressure: ";
    YamlStreamer< ::uavcan::equipment::air_data::StaticPressure::FieldTypes::static_pressure >::stream(s, obj.static_pressure, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "static_pressure_variance: ";
    YamlStreamer< ::uavcan::equipment::air_data::StaticPressure::FieldTypes::static_pressure_variance >::stream(s, obj.static_pressure_variance, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace air_data
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::air_data::StaticPressure::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::air_data::StaticPressure >::stream(s, obj, 0);
    return s;
}

} // Namespace air_data
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_HPP_INCLUDED