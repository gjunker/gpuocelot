#include "PTXParserState.h"
#include "PTXParseException.h"
#include <sstream>

namespace parser {
    class PTXLexer;
}

#include <ptxgrammar.hpp>

// Preprocessor Macros
#define throw_exception( messageData, type ) \
    {\
        std::stringstream error;\
        error << messageData;\
        parser::PTXParseException exception;\
        exception.error = parser::PTXParseException::type;\
        exception.message = error.str();\
        throw exception;\
    }

namespace {

ir::PTXOperand::DataType
tokenToDataType( int token )
{
    switch( token )
    {
        case TOKEN_U8:   return ir::PTXOperand::u8; break;
        case TOKEN_U16:  return ir::PTXOperand::u16; break;
        case TOKEN_U32:  return ir::PTXOperand::u32; break;
        case TOKEN_U64:  return ir::PTXOperand::u64; break;
        case TOKEN_S8:   return ir::PTXOperand::s8; break;
        case TOKEN_S16:  return ir::PTXOperand::s16; break;
        case TOKEN_S32:  return ir::PTXOperand::s32; break;
        case TOKEN_S64:  return ir::PTXOperand::s64; break;
        case TOKEN_B8:   return ir::PTXOperand::b8; break;
        case TOKEN_B16:  return ir::PTXOperand::b16; break;
        case TOKEN_B32:  return ir::PTXOperand::b32; break;
        case TOKEN_B64:  return ir::PTXOperand::b64; break;
        case TOKEN_PRED: return ir::PTXOperand::pred; break;
        case TOKEN_F16:  return ir::PTXOperand::f16; break;
        case TOKEN_F32:  return ir::PTXOperand::f32; break;
        case TOKEN_F64:  return ir::PTXOperand::f64; break;
        default:
        {
            assertM(false, "Parsed invalid data type");
            break;
        }
    }

    return ir::PTXOperand::TypeSpecifier_invalid;
}

ir::PTXOperand::Vec
tokenToVec( int token )
{
    switch( token )
    {
        case TOKEN_V2: return ir::PTXOperand::v2; break;
        case TOKEN_V4: return ir::PTXOperand::v4; break;
        default:
        {
            parser::PTXParseException exception;
            exception.error = parser::PTXParseException::InvalidVecType;

            std::stringstream stream;
            stream << token;
            exception.message = "Got invalid vector type " + stream.str();
            throw exception;
            break;
        }
    }

    return ir::PTXOperand::v1;
}

ir::PTXOperand::VectorIndex
tokenToVectorIndex( int token )
{
    switch( token )
    {
        case TOKEN_X: return ir::PTXOperand::ix;
        case TOKEN_Y: return ir::PTXOperand::iy;
        case TOKEN_Z: return ir::PTXOperand::iz;
        case TOKEN_W: return ir::PTXOperand::iw;
    }
    return ir::PTXOperand::ix;
}

ir::PTXInstruction::Opcode
stringToOpcode( std::string string )
{
    if( string == "abs" ) return ir::PTXInstruction::Abs;
    if( string == "add" ) return ir::PTXInstruction::Add;
    if( string == "addc" ) return ir::PTXInstruction::AddC;
    if( string == "and" ) return ir::PTXInstruction::And;
    if( string == "atom" ) return ir::PTXInstruction::Atom;
    if( string == "bar.sync" ) return ir::PTXInstruction::Bar;
    if( string == "bar" ) return ir::PTXInstruction::Bar;
    if( string == "bfe" ) return ir::PTXInstruction::Bfe;
    if( string == "bfi" ) return ir::PTXInstruction::Bfi;
    if( string == "bfind" ) return ir::PTXInstruction::Bfind;
    if( string == "bra" ) return ir::PTXInstruction::Bra;
    if( string == "brev" ) return ir::PTXInstruction::Brev;
    if( string == "brkpt" ) return ir::PTXInstruction::Brkpt;
    if( string == "call" ) return ir::PTXInstruction::Call;
    if( string == "clz" ) return ir::PTXInstruction::Clz;
    if( string == "cnot" ) return ir::PTXInstruction::CNot;
    if( string == "copysign" ) return ir::PTXInstruction::CopySign;
    if( string == "cos" ) return ir::PTXInstruction::Cos;
    if( string == "cvt" ) return ir::PTXInstruction::Cvt;
    if( string == "cvta" ) return ir::PTXInstruction::Cvta;
    if( string == "div" ) return ir::PTXInstruction::Div;
    if( string == "ex2" ) return ir::PTXInstruction::Ex2;
    if( string == "exit" ) return ir::PTXInstruction::Exit;
    if( string == "fma" ) return ir::PTXInstruction::Fma;
    if( string == "isspacep" ) return ir::PTXInstruction::Isspacep;
    if( string == "ld" ) return ir::PTXInstruction::Ld;
    if( string == "ldu" ) return ir::PTXInstruction::Ldu;
    if( string == "lg2" ) return ir::PTXInstruction::Lg2;
    if( string == "mad24" ) return ir::PTXInstruction::Mad24;
    if( string == "mad" ) return ir::PTXInstruction::Mad;
    if( string == "madc" ) return ir::PTXInstruction::MadC;
    if( string == "max" ) return ir::PTXInstruction::Max;
    if( string == "membar" ) return ir::PTXInstruction::Membar;
    if( string == "min" ) return ir::PTXInstruction::Min;
    if( string == "mov" ) return ir::PTXInstruction::Mov;
    if( string == "mul24" ) return ir::PTXInstruction::Mul24;
    if( string == "mul" ) return ir::PTXInstruction::Mul;
    if( string == "neg" ) return ir::PTXInstruction::Neg;
    if( string == "not" ) return ir::PTXInstruction::Not;
    if( string == "pmevent" ) return ir::PTXInstruction::Pmevent;
    if( string == "popc" ) return ir::PTXInstruction::Popc;
    if( string == "prmt" ) return ir::PTXInstruction::Prmt;
    if( string == "or" ) return ir::PTXInstruction::Or;
    if( string == "rcp" ) return ir::PTXInstruction::Rcp;
    if( string == "red" ) return ir::PTXInstruction::Red;
    if( string == "rem" ) return ir::PTXInstruction::Rem;
    if( string == "ret" ) return ir::PTXInstruction::Ret;
    if( string == "rsqrt" ) return ir::PTXInstruction::Rsqrt;
    if( string == "sad" ) return ir::PTXInstruction::Sad;
    if( string == "selp" ) return ir::PTXInstruction::SelP;
    if( string == "set" ) return ir::PTXInstruction::Set;
    if( string == "setp" ) return ir::PTXInstruction::SetP;
    if( string == "shl" ) return ir::PTXInstruction::Shl;
    if( string == "shfl" ) return ir::PTXInstruction::Shfl;
    if( string == "shr" ) return ir::PTXInstruction::Shr;
    if( string == "sin" ) return ir::PTXInstruction::Sin;
    if( string == "slct" ) return ir::PTXInstruction::SlCt;
    if( string == "sqrt" ) return ir::PTXInstruction::Sqrt;
    if( string == "st" ) return ir::PTXInstruction::St;
    if( string == "sub" ) return ir::PTXInstruction::Sub;
    if( string == "subc" ) return ir::PTXInstruction::SubC;
    if( string == "suld" ) return ir::PTXInstruction::Suld;
    if( string == "sust" ) return ir::PTXInstruction::Sust;
    if( string == "sured" ) return ir::PTXInstruction::Sured;
    if( string == "suq" ) return ir::PTXInstruction::Suq;
    if( string == "tex" ) return ir::PTXInstruction::Tex;
    if( string == "testp" ) return ir::PTXInstruction::TestP;
    if( string == "tld4" ) return ir::PTXInstruction::Tld4;
    if( string == "trap" ) return ir::PTXInstruction::Trap;
    if( string == "txq" ) return ir::PTXInstruction::Txq;
    if( string == "vote" ) return ir::PTXInstruction::Vote;
    if( string == "xor" ) return ir::PTXInstruction::Xor;

    return ir::PTXInstruction::Nop;
}


ir::PTXInstruction::Modifier
tokenToModifier( int token )
{
    switch( token )
    {
        case TOKEN_HI: return ir::PTXInstruction::hi; break;
        case TOKEN_LO: return ir::PTXInstruction::lo; break;
        case TOKEN_WIDE: return ir::PTXInstruction::wide; break;
        case TOKEN_SAT: return ir::PTXInstruction::sat; break;
        case TOKEN_RNI: return ir::PTXInstruction::rni; break;
        case TOKEN_RN: return ir::PTXInstruction::rn; break;
        case TOKEN_RZI: return ir::PTXInstruction::rzi; break;
        case TOKEN_RZ: return ir::PTXInstruction::rz; break;
        case TOKEN_RMI: return ir::PTXInstruction::rmi; break;
        case TOKEN_RM: return ir::PTXInstruction::rm; break;
        case TOKEN_RPI: return ir::PTXInstruction::rpi; break;
        case TOKEN_RP: return ir::PTXInstruction::rp; break;
        case TOKEN_FTZ: return ir::PTXInstruction::ftz; break;
        case TOKEN_APPROX: return ir::PTXInstruction::approx; break;
        default: break;
    }

    return ir::PTXInstruction::Modifier_invalid;
}

ir::PTXInstruction::AddressSpace
tokenToAddressSpace( int token )
{
    switch( token )
    {
        case TOKEN_CONST: return ir::PTXInstruction::Const; break;
        case TOKEN_GLOBAL: return ir::PTXInstruction::Global; break;
        case TOKEN_LOCAL: return ir::PTXInstruction::Local; break;
        case TOKEN_PARAM: return ir::PTXInstruction::Param; break;
        case TOKEN_SHARED: return ir::PTXInstruction::Shared; break;
        default: break;
    }

    return ir::PTXInstruction::AddressSpace_Invalid;
}

ir::PTXStatement::Directive
tokenToDirective( int token )
{
    switch( token )
    {
        case TOKEN_CONST: return ir::PTXStatement::Const; break;
        case TOKEN_GLOBAL: return ir::PTXStatement::Global; break;
        case TOKEN_LOCAL: return ir::PTXStatement::Local; break;
        case TOKEN_PARAM: return ir::PTXStatement::Param; break;
        case TOKEN_SHARED: return ir::PTXStatement::Shared; break;
        default: break;
    }

    return ir::PTXStatement::Directive_invalid;
}

ir::PTXInstruction::ReductionOperation
tokenToReductionOperation( int token )
{
    switch( token )
    {
        case TOKEN_AND: return ir::PTXInstruction::ReductionAnd; break;
        case TOKEN_XOR: return ir::PTXInstruction::ReductionXor; break;
        case TOKEN_OR: return ir::PTXInstruction::ReductionOr; break;
        case TOKEN_ADD: return ir::PTXInstruction::ReductionAdd; break;
        case TOKEN_INC: return ir::PTXInstruction::ReductionInc; break;
        case TOKEN_DEC: return ir::PTXInstruction::ReductionDec; break;
        case TOKEN_MIN: return ir::PTXInstruction::ReductionMin; break;
        case TOKEN_MAX: return ir::PTXInstruction::ReductionMax; break;
        case TOKEN_POPC: return ir::PTXInstruction::ReductionPopc; break;
        default: break;
    }

    return ir::PTXInstruction::ReductionOperation_Invalid;
}

ir::PTXInstruction::AtomicOperation
tokenToAtomicOperation( int token )
{
    switch( token )
    {
        case TOKEN_AND: return ir::PTXInstruction::AtomicAnd; break;
        case TOKEN_XOR: return ir::PTXInstruction::AtomicXor; break;
        case TOKEN_OR: return ir::PTXInstruction::AtomicOr; break;
        case TOKEN_ADD: return ir::PTXInstruction::AtomicAdd; break;
        case TOKEN_INC: return ir::PTXInstruction::AtomicInc; break;
        case TOKEN_DEC: return ir::PTXInstruction::AtomicDec; break;
        case TOKEN_MIN: return ir::PTXInstruction::AtomicMin; break;
        case TOKEN_MAX: return ir::PTXInstruction::AtomicMax; break;
        case TOKEN_CAS: return ir::PTXInstruction::AtomicCas; break;
        case TOKEN_EXCH: return ir::PTXInstruction::AtomicExch; break;
        default: break;
    }

    return ir::PTXInstruction::AtomicOperation_Invalid;
}

ir::PTXInstruction::CmpOp
tokenToCmpOp( int token )
{
    switch( token )
    {
        case TOKEN_EQ: return ir::PTXInstruction::Eq; break;
        case TOKEN_NE: return ir::PTXInstruction::Ne; break;
        case TOKEN_LT: return ir::PTXInstruction::Lt; break;
        case TOKEN_LE: return ir::PTXInstruction::Le; break;
        case TOKEN_GT: return ir::PTXInstruction::Gt; break;
        case TOKEN_GE: return ir::PTXInstruction::Ge; break;
        case TOKEN_LO: return ir::PTXInstruction::Lo; break;
        case TOKEN_LS: return ir::PTXInstruction::Ls; break;
        case TOKEN_HI: return ir::PTXInstruction::Hi; break;
        case TOKEN_HS: return ir::PTXInstruction::Hs; break;
        case TOKEN_EQU: return ir::PTXInstruction::Equ; break;
        case TOKEN_NEU: return ir::PTXInstruction::Neu; break;
        case TOKEN_LTU: return ir::PTXInstruction::Ltu; break;
        case TOKEN_LEU: return ir::PTXInstruction::Leu; break;
        case TOKEN_GTU: return ir::PTXInstruction::Gtu; break;
        case TOKEN_GEU: return ir::PTXInstruction::Geu; break;
        case TOKEN_NUM: return ir::PTXInstruction::Num; break;
        case TOKEN_NAN: return ir::PTXInstruction::Nan; break;
        default: break;
    }

    return ir::PTXInstruction::CmpOp_Invalid;
}

ir::PTXInstruction::CacheOperation
tokenToCacheOperation(int token)
{
    switch (token)
    {
        case TOKEN_CA: return ir::PTXInstruction::Ca;
        case TOKEN_WB: return ir::PTXInstruction::Wb;
        case TOKEN_CG: return ir::PTXInstruction::Cg;
        case TOKEN_CS: return ir::PTXInstruction::Cs;
        case TOKEN_CV: return ir::PTXInstruction::Cv;
        case TOKEN_WT: return ir::PTXInstruction::Wt;
        case TOKEN_NC: return ir::PTXInstruction::Nc;
        default: break;
    }
    return ir::PTXInstruction::CacheOperation_Invalid;
}

ir::PTXInstruction::CacheLevel
tokenToCacheLevel(int token) {
    switch (token) {
        case TOKEN_L1: return ir::PTXInstruction::L1;
        case TOKEN_L2: return ir::PTXInstruction::L2;
        default: break;
    }
    return ir::PTXInstruction::CacheLevel_invalid;
}

ir::PTXInstruction::ClampOperation
tokenToClampOperation(int token) {
    switch (token)
    {
        case TOKEN_TRAP: return ir::PTXInstruction::TrapOOB;
        case TOKEN_CLAMP: return ir::PTXInstruction::Clamp;
        case TOKEN_ZERO: return ir::PTXInstruction::Zero;
        default: break;
    }
    return ir::PTXInstruction::ClampOperation_Invalid;
}

ir::PTXInstruction::FormatMode
tokenToFormatMode(int token)
{
    switch (token)
    {
        case TOKEN_B: return ir::PTXInstruction::Unformatted;
        case TOKEN_P: return ir::PTXInstruction::Formatted;
        default: break;
    }
    return ir::PTXInstruction::FormatMode_Invalid;
}

ir::PTXInstruction::SurfaceQuery
tokenToSurfaceQuery(int token)
{
    report("tokenToSurfaceQuery(" << token << ")");

    switch (token)
    {
        case TOKEN_WIDTH: return ir::PTXInstruction::Width;
        case TOKEN_HEIGHT: return ir::PTXInstruction::Height;
        case TOKEN_DEPTH: return ir::PTXInstruction::Depth;
        case TOKEN_CHANNEL_DATA_TYPE: return ir::PTXInstruction::ChannelDataType;
        case TOKEN_CHANNEL_ORDER: return ir::PTXInstruction::ChannelOrder;
        case TOKEN_NORMALIZED_COORDS: return ir::PTXInstruction::NormalizedCoordinates;
        case TOKEN_FILTER_MODE: return ir::PTXInstruction::SamplerFilterMode;
        case TOKEN_ADDR_MODE_0: return ir::PTXInstruction::SamplerAddrMode0;
        case TOKEN_ADDR_MODE_1: return ir::PTXInstruction::SamplerAddrMode1;
        case TOKEN_ADDR_MODE_2: return ir::PTXInstruction::SamplerAddrMode2;
        default: break;
    }
    return ir::PTXInstruction::SurfaceQuery_Invalid;
}

ir::PTXInstruction::ColorComponent
tokenToColorComponent(
    int token)
{
    switch (token)
    {
        case TOKEN_R: return ir::PTXInstruction::red;
        case TOKEN_G: return ir::PTXInstruction::green;
        case TOKEN_B: return ir::PTXInstruction::blue;
        case TOKEN_A: return ir::PTXInstruction::alpha;
        default: break;
    }
    return ir::PTXInstruction::ColorComponent_Invalid;
}

ir::PTXInstruction::BarrierOperation
tokenToBarrierOp(int token)
{
    switch( token )
    {
        case TOKEN_ARRIVE: return ir::PTXInstruction::BarArrive;
        case TOKEN_RED: return ir::PTXInstruction::BarReduction;
        default:
            break;
    }
    return ir::PTXInstruction::BarSync;
}

ir::PTXInstruction::BoolOp
tokenToBoolOp( int token )
{
    switch( token )
    {
        case TOKEN_AND: return ir::PTXInstruction::BoolAnd; break;
        case TOKEN_OR: return ir::PTXInstruction::BoolOr; break;
        case TOKEN_XOR: return ir::PTXInstruction::BoolXor; break;
        default: break;

    }

    return ir::PTXInstruction::BoolOp_Invalid;
}

ir::PTXInstruction::Geometry
tokenToGeometry( int token )
{
    switch( token )
    {
        case TOKEN_1D: return ir::PTXInstruction::_1d; break;
        case TOKEN_2D: return ir::PTXInstruction::_2d; break;
        case TOKEN_3D: return ir::PTXInstruction::_3d; break;
        case TOKEN_A1D: return ir::PTXInstruction::_a1d; break;
        case TOKEN_A2D: return ir::PTXInstruction::_a2d; break;
        case TOKEN_CUBE: return ir::PTXInstruction::_cube; break;
        case TOKEN_ACUBE: return ir::PTXInstruction::_acube; break;
        default: break;
    }

    return ir::PTXInstruction::Geometry_Invalid;
}

ir::PTXInstruction::VoteMode
tokenToVoteMode( int token )
{
    switch( token )
    {
        case TOKEN_ANY:    return ir::PTXInstruction::Any;    break;
        case TOKEN_ALL:    return ir::PTXInstruction::All;    break;
        case TOKEN_UNI:    return ir::PTXInstruction::Uni;    break;
        case TOKEN_BALLOT: return ir::PTXInstruction::Ballot; break;
        default: break;
    }

    return ir::PTXInstruction::VoteMode_Invalid;
}

ir::PTXInstruction::ShuffleMode
tokenToShuffleMode( int token )
{
    switch( token )
    {
        case TOKEN_UP:   return ir::PTXInstruction::Up;   break;
        case TOKEN_DOWN: return ir::PTXInstruction::Down; break;
        case TOKEN_BFLY: return ir::PTXInstruction::Bfly; break;
        case TOKEN_IDX:  return ir::PTXInstruction::Idx;  break;
        default: break;
    }

    return ir::PTXInstruction::ShuffleMode_Invalid;
}

ir::PTXInstruction::Level
tokenToLevel( int token )
{
    switch( token )
    {
        case TOKEN_CTA: return ir::PTXInstruction::CtaLevel; break;
        case TOKEN_GL: return ir::PTXInstruction::GlobalLevel; break;
        case TOKEN_SYS: return ir::PTXInstruction::SystemLevel; break;
        default: break;
    }

    return ir::PTXInstruction::Level_Invalid;
}

ir::PTXInstruction::PermuteMode
tokenToPermuteMode( int token )
{
    switch( token )
    {
        case TOKEN_F4E: return ir::PTXInstruction::ForwardFourExtract;
            break;
        case TOKEN_B4E: return ir::PTXInstruction::BackwardFourExtract;
            break;
        case TOKEN_RC8: return ir::PTXInstruction::ReplicateEight;    break;
        case TOKEN_ECL: return ir::PTXInstruction::EdgeClampLeft;     break;
        case TOKEN_ECR: return ir::PTXInstruction::EdgeClampRight;    break;
        case TOKEN_RC16: return ir::PTXInstruction::ReplicateSixteen; break;
            break;
        default: break;
    }

    return ir::PTXInstruction::DefaultPermute;
}

ir::PTXInstruction::FloatingPointMode
tokenToFloatingPointMode( int token )
{
    switch( token )
    {
        case TOKEN_FINITE:    return ir::PTXInstruction::Finite;    break;
        case TOKEN_INFINITE:  return ir::PTXInstruction::Infinite;  break;
        case TOKEN_NUMBER:    return ir::PTXInstruction::Number;    break;
        case TOKEN_NOT_A_NUMBER:
            return ir::PTXInstruction::NotANumber; break;
        case TOKEN_NORMAL:    return ir::PTXInstruction::Normal;    break;
        case TOKEN_SUBNORMAL: return ir::PTXInstruction::SubNormal; break;
        default: break;
    }

    return ir::PTXInstruction::FloatingPointMode_Invalid;
}

ir::PTXStatement::TextureSpace
tokenToTextureSpace( int token )
{
    switch( token )
    {
        case TOKEN_GLOBAL: return ir::PTXStatement::GlobalSpace;    break;
        case TOKEN_PARAM:  return ir::PTXStatement::ParameterSpace; break;
    }
    return ir::PTXStatement::InvalidSpace;
}

ir::PTXOperand::DataType
smallestType( long long int value )
{
    return ir::PTXOperand::s64;
}

ir::PTXOperand::DataType
smallestType( long long unsigned int value )
{
    return ir::PTXOperand::u64;
}

} // namespace

namespace parser {

std::string
toString( YYLTYPE& location, parser::PTXParserState& state )
{
    std::stringstream stream;
    stream << state.fileName << " (" << location.first_line << ", "
        << location.first_column << "): ";
    return stream.str();
}

PTXParserState::OperandWrapper::OperandWrapper( const ir::PTXOperand& o,
    ir::PTXInstruction::AddressSpace s ) : operand( o ), space( s )
{

}

void PTXParserState::FunctionPrototype::clear()
{
    returnTypes.clear();
    argumentTypes.clear();
    name.clear();
}

bool PTXParserState::FunctionPrototype::compare(
    const FunctionPrototype& t )
{
    if( t.returnTypes.size()   != returnTypes.size()   ) return false;
    if( t.argumentTypes.size() != argumentTypes.size() ) return false;

    for( TypeVector::const_iterator fromType = returnTypes.begin(),
        toType = t.returnTypes.begin(); fromType != returnTypes.end();
        ++fromType, ++toType )
    {
        if( !ir::PTXOperand::relaxedValid( *fromType, *toType ) )
        {
            return false;
        }
    }

    for( TypeVector::const_iterator fromType = t.argumentTypes.begin(),
        toType = argumentTypes.begin(); fromType != t.argumentTypes.end();
        ++fromType, ++toType )
    {
        if( !ir::PTXOperand::relaxedValid( *toType, *fromType ) )
        {
            return false;
        }
    }

    return true;
}

std::string PTXParserState::FunctionPrototype::toString() const
{
    std::stringstream stream;

    stream << "(";
    for( TypeVector::const_iterator type = returnTypes.begin();
        type != returnTypes.end(); ++type )
    {
        if( type != returnTypes.begin() )
        {
            stream << ", ";
        }
        stream << ir::PTXOperand::toString( *type );
    }

    stream << ") " << name << " (";

    for( TypeVector::const_iterator type = argumentTypes.begin();
        type != argumentTypes.end(); ++type )
    {
        if( type != argumentTypes.begin() )
        {
            stream << ", ";
        }
        stream << ir::PTXOperand::toString( *type );
    }
    stream << ")";

    return stream.str();
}

PTXParserState::Context::Context( unsigned int i )
: id(i), instructionCount( 0 )
{

}

ir::PTXInstruction::AddressSpace PTXParserState::_toAddressSpace(
    ir::PTXStatement::Directive directive )
{
    switch( directive )
    {
        case ir::PTXStatement::Shared : return ir::PTXInstruction::Shared;
        case ir::PTXStatement::Local  : return ir::PTXInstruction::Local;
        case ir::PTXStatement::Param  : return ir::PTXInstruction::Param;
        case ir::PTXStatement::Global : return ir::PTXInstruction::Global;
        case ir::PTXStatement::Const  : return ir::PTXInstruction::Const;
        default: break;
    }
    return ir::PTXInstruction::AddressSpace_Invalid;
}

void PTXParserState::_setImmediateTypes()
{
    ir::PTXInstruction& instruction = statement.instruction;

    ir::PTXOperand* sources[] =
        { &instruction.a, &instruction.b, &instruction.c };

    for( unsigned int i = 0; i < 3; ++i )
    {
        ir::PTXOperand& operand = *sources[i];

        if( operand.addressMode == ir::PTXOperand::Immediate )
        {
            operand.type = instruction.type;
        }
    }
}

static std::string strip(const std::string& name)
{
    if(name.find("_Zcontext_") == 0)
    {
        size_t position = name.find("_", 10);

        return name.substr(position + 1);
    }

    return name;
}

std::string PTXParserState::_nameInContext(
    const std::string& name )
{
    // Don't rename in the original context
    if( contexts.size() <= 3 ) return name;

    // Allocate a new context for this value if here is a collision
    bool collision = false;

    for( OperandMap::iterator operand = contexts.back().operands.begin();
        operand != contexts.back().operands.end(); ++operand )
    {
        if( strip( operand->first ) == strip( name ) )
        {
            collision = true;
            break;
        }
    }

    std::stringstream stream;

    if( collision )
    {
        // Allocate a new name in a new context
        stream << "_Zcontext_" << contextId++ << "_" << strip( name );
    }
    else
    {
        // Allocate a new name in the current context
        stream << "_Zcontext_" << contexts.back().id
            << "_" << strip( name );
    }

    return stream.str();
}

PTXParserState::OperandWrapper*
    PTXParserState::_getOperand( const std::string& name )
{
    for( ContextStack::reverse_iterator context = contexts.rbegin();
        context != contexts.rend(); ++context )
    {
        OperandMap::iterator operand = context->operands.find( name );

        if( operand != context->operands.end() ) return &operand->second;

        operand = context->operands.find( strip( name ) );

        if( operand != context->operands.end() ) return &operand->second;
    }

    return 0;
}

PTXParserState::OperandWrapper*
    PTXParserState::_getOperandInScope( const std::string& name )
{
    assert( !contexts.empty() );

    OperandMap::iterator operand = contexts.back().operands.find( name );

    if( operand != contexts.back().operands.end() ) return &operand->second;

    operand = contexts.back().operands.find( strip( name ) );

    if( operand != contexts.back().operands.end() ) return &operand->second;

    return 0;
}

PTXParserState::FunctionPrototype* PTXParserState::_getPrototype(
    const std::string& name )
{
    for( ContextStack::reverse_iterator context = contexts.rbegin();
        context != contexts.rend(); ++context )
    {
        PrototypeMap::iterator prototype = context->prototypes.find( name );

        if( prototype != context->prototypes.end() )
        {
            return &prototype->second;
        }
    }

    return 0;
}

static ir::PTXOperand::SpecialRegister envReg( unsigned int i )
{
    switch( i )
    {
        case 0: return ir::PTXOperand::envreg0;
        case 1: return ir::PTXOperand::envreg1;
        case 2: return ir::PTXOperand::envreg2;
        case 3: return ir::PTXOperand::envreg3;
        case 4: return ir::PTXOperand::envreg4;
        case 5: return ir::PTXOperand::envreg5;
        case 6: return ir::PTXOperand::envreg6;
        case 7: return ir::PTXOperand::envreg7;
        case 8: return ir::PTXOperand::envreg8;
        case 9: return ir::PTXOperand::envreg9;
        case 10: return ir::PTXOperand::envreg10;
        case 11: return ir::PTXOperand::envreg11;
        case 12: return ir::PTXOperand::envreg12;
        case 13: return ir::PTXOperand::envreg13;
        case 14: return ir::PTXOperand::envreg14;
        case 15: return ir::PTXOperand::envreg15;
        case 16: return ir::PTXOperand::envreg16;
        case 17: return ir::PTXOperand::envreg17;
        case 18: return ir::PTXOperand::envreg18;
        case 19: return ir::PTXOperand::envreg19;
        case 20: return ir::PTXOperand::envreg20;
        case 21: return ir::PTXOperand::envreg21;
        case 22: return ir::PTXOperand::envreg22;
        case 23: return ir::PTXOperand::envreg23;
        case 24: return ir::PTXOperand::envreg24;
        case 25: return ir::PTXOperand::envreg25;
        case 26: return ir::PTXOperand::envreg26;
        case 27: return ir::PTXOperand::envreg27;
        case 28: return ir::PTXOperand::envreg28;
        case 29: return ir::PTXOperand::envreg29;
        case 30: return ir::PTXOperand::envreg30;
        case 31: return ir::PTXOperand::envreg31;
        default: break;
    }
    return ir::PTXOperand::SpecialRegister_invalid;
}

void PTXParserState::addSpecialRegisters()
{
    OperandMap& operands = contexts.back().operands;

    operands.insert( std::make_pair( "%tid", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::tid, ir::PTXOperand::iAll ) ) ) );
    operands.insert( std::make_pair( "%ntid", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::ntid, ir::PTXOperand::iAll ) ) ) );
    operands.insert( std::make_pair( "%laneid", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::laneId, ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%nwarpid", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::nwarpId, ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "WARP_SZ", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::warpSize, ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%ctaid", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::ctaId, ir::PTXOperand::iAll ) ) ) );
    operands.insert( std::make_pair( "%nctaid", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::nctaId,
        ir::PTXOperand::iAll ) ) ) );
    operands.insert( std::make_pair( "%smid", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::smId, ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%nsmid", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::nsmId, ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%gridId", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::gridId, ir::PTXOperand::ix ) ) ) );

    operands.insert( std::make_pair( "%lanemask_eq", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::lanemask_eq,
        ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%lanemask_le", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::lanemask_le,
        ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%lanemask_lt", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::lanemask_lt,
        ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%lanemask_ge", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::lanemask_ge,
        ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%lanemask_gt", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::lanemask_gt,
        ir::PTXOperand::ix ) ) ) );

    operands.insert( std::make_pair( "%clock", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::clock, ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%clock64", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::clock64, ir::PTXOperand::ix,
        ir::PTXOperand::u64 ) ) ) );

    operands.insert( std::make_pair( "%pm0", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::pm0, ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%pm1", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::pm1, ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%pm2", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::pm2, ir::PTXOperand::ix ) ) ) );
    operands.insert( std::make_pair( "%pm3", OperandWrapper(
        ir::PTXOperand( ir::PTXOperand::pm3, ir::PTXOperand::ix ) ) ) );

    for( unsigned int i = 0; i < 32; ++i )
    {
        std::stringstream stream;
        stream << "%envreg" << i;

        operands.insert( std::make_pair( stream.str(),
            OperandWrapper( ir::PTXOperand( envReg( i ), ir::PTXOperand::ix,
            ir::PTXOperand::b32 ) ) ) );
    }
}

void PTXParserState::maxnreg( unsigned int regs )
{
    report( "  Rule: TOKEN_MAXNREG TOKEN_DECIMAL_CONSTANT" );
    statement.directive = ir::PTXStatement::Maxnreg;
}

void PTXParserState::maxntid( unsigned int tidx, unsigned int tidy,
    unsigned int tidz )
{
    report( "  Rule: TOKEN_MAXNTID TOKEN_DECIMAL_CONSTANT" );
    statement.directive = ir::PTXStatement::Maxntid;

}

void PTXParserState::ctapersm( int target, unsigned int ctas )
{
    report( "  Rule: shareModel ':' TOKEN_DECIMAL_CONSTANT" );
}

void PTXParserState::maxnctapersm( unsigned int ctas )
{
    report( "  Rule: TOKEN_MAXNCTAPERSM TOKEN_DECIMAL_CONSTANT" );
    ir::PTXStatement::Data data;
    data.u32 = ctas;
    statement.directive = ir::PTXStatement::Maxnctapersm;
    statement.array.values.push_back(data);
}

void PTXParserState::maxnctapersm()
{
    report( "  Rule: TOKEN_MAXNCTAPERSM ctapersmList" );
    statement.directive = ir::PTXStatement::Maxnctapersm;
}

void PTXParserState::minnctapersm( unsigned int ctas )
{
    report( "  Rule: TOKEN_MINNCTAPERSM TOKEN_DECIMAL_CONSTANT" );
    ir::PTXStatement::Data data;
    data.u32 = ctas;
    statement.directive = ir::PTXStatement::Minnctapersm;
    statement.array.values.push_back(data);
}

void PTXParserState::minnctapersm()
{
    report( "  Rule: TOKEN_MINNCTAPERSM ctapersmList" );
    statement.directive = ir::PTXStatement::Minnctapersm;
}

void PTXParserState::preprocessor( int token )
{
    throw_exception( "PTX preprocessor commands not supported in Ocelot.",
        NotSupported );
}

void PTXParserState::version( double version, YYLTYPE& location )
{
    report( "  Rule: VERSION DOUBLE_CONSTANT" );

    std::stringstream stream1;
    stream1.setf( std::ios::fixed, std::ios::floatfield );
    stream1.precision( 1 );
    stream1 << version;
    std::string value( stream1.str() );

    std::stringstream stream2;
    std::string::iterator fi = value.begin();

    for( ; fi != value.end(); ++fi )
    {
        if( *fi == '.' )
        {
            ++fi;
            break;
        }
        stream2 << *fi;
    }

    if( fi == value.end() )
    {
        throw_exception( toString( location, *this )
            << "Malformed version number " << value, MalformedVersion );
    }

    statement.directive = ir::PTXStatement::Version;
    stream2 >> statement.major;

    std::stringstream stream3;

    for( ; fi != value.end(); ++fi )
    {
        stream3 << *fi;
    }

    if( stream3.str().empty() )
    {
        throw_exception( toString( location, *this )
            << "Malformed version number " << value, MalformedVersion );
    }

    stream3 >> statement.minor;

    if(statement.major < 2 )
    {
        throw_exception( toString( location, *this )
            << "Cannot parse PTX version " << statement.major
            << "." << statement.minor << " with version 2.3 parser.\n"
            << " Consider recompiling your CUDA source with `nvcc -arch=sm_20`",
            NotVersion2_1 );
    }
}

void PTXParserState::addressSize( unsigned int size )
{
    report( "  Rule: ADDRESS_SIZE DECIMAL_CONSTANT" );
    statement.directive = ir::PTXStatement::AddressSize;
    statement.addressSize = size;
}

void PTXParserState::identifierList( const std::string& identifier )
{
    report( "  Rule: IDENTIFIER" );
    identifiers.clear();
    report( "   Appending " << identifier << " to list." );
    identifiers.push_back( identifier );
}

void PTXParserState::identifierList2( const std::string& identifier )
{
    report( "  Rule: identifierList , IDENTIFIER" );
    report( "   Appending " << identifier << " to list." );
    identifiers.push_back( identifier );
}

void PTXParserState::decimalListSingle( long long int value )
{
    report( "  Rule: DECIMAL_CONSTANT" );

    if( statement.array.stride.empty() )
    {
        statement.array.stride.push_back( 1 );
    }

    report( "   Appending " << value << " to decimal list.");
    ir::PTXStatement::Data data;
    data.s64 = value;
    statement.array.values.push_back( data );
}

void PTXParserState::decimalListSingle2( long long int value )
{
    report( "  Rule: decimalList ',' DECIMAL_CONSTANT" );

    report( "   Appending " << value << " to decimal list.");
    ir::PTXStatement::Data data;
    data.s64 = value;
    statement.array.values.push_back( data );
}

void PTXParserState::symbolListSingle( const std::string& identifier )
{
    report( "  Rule: identifier" );
    unsigned int index = statement.array.values.size() - 1;

    report( "   Appending " << identifier << ", " << index
        << " to symbol list.");

    statement.array.symbols.push_back(
        ir::PTXStatement::Symbol( identifier, index ) );
}

void PTXParserState::symbolListSingle2( const std::string& identifier )
{
    report( "  Rule: decimalList ',' identifier" );
    unsigned int index = statement.array.values.size() - 1;

    report( "   Appending " << identifier << ", " << index
        << " to symbol list.");

    statement.array.symbols.push_back(
        ir::PTXStatement::Symbol( identifier, index ) );
}

void PTXParserState::floatList( double value )
{
    report( "  Rule: DOUBLE_CONSTANT" );

    if( statement.array.stride.empty() )
    {
        statement.array.stride.push_back( 1 );
    }

    report( "   Appending " << value << " to float list.");
    ir::PTXStatement::Data data;
    data.f64 = value;

    statement.array.values.push_back( data );
}

void PTXParserState::floatList1( double value )
{
    report( "  Rule: floatList ',' DOUBLE_CONSTANT" );
    report( "   Appending " << value << " to float list.");
    ir::PTXStatement::Data data;
    data.f64 = value;
    statement.array.values.push_back( data );
}

void PTXParserState::singleList( float value )
{
    report( "  Rule: DOUBLE_CONSTANT" );

    if( statement.array.stride.empty() )
    {
        statement.array.stride.push_back( 1 );
    }

    report( "   Appending " << value << " to single list.");
    ir::PTXStatement::Data data;
    data.f32 = value;
    statement.array.values.push_back( data );
}

void PTXParserState::singleList1( float value )
{
    report( "  Rule: singleList ',' SINGLE_CONSTANT" );

    report( "   Appending " << value << " to single list.");
    ir::PTXStatement::Data data;
    data.f32 = value;
    statement.array.values.push_back( data );
}

void PTXParserState::targetElement( int token )
{
    report( "  Rule: targetOption" );
    if( token == TOKEN_SM10 ) statement.targets.push_back( "sm_10" );
    else if( token == TOKEN_SM11 ) statement.targets.push_back( "sm_11" );
    else if( token == TOKEN_SM12 ) statement.targets.push_back( "sm_12" );
    else if( token == TOKEN_SM13 ) statement.targets.push_back( "sm_13" );
    else if( token == TOKEN_SM20 ) statement.targets.push_back( "sm_20" );
    else if( token == TOKEN_SM21 ) statement.targets.push_back( "sm_21" );
    else if( token == TOKEN_SM30 ) statement.targets.push_back( "sm_30" );
    else if( token == TOKEN_SM35 ) statement.targets.push_back( "sm_35" );
    else if( token == TOKEN_MAP_F64_TO_F32 )
    {
        statement.targets.push_back( "map_f64_to_f32" );
    }
    else if (token == TOKEN_TEXMODE_INDEPENDENT) {
        statement.targets.push_back("texmode_independent");
    }
    else if (token == TOKEN_TEXMODE_UNIFIED) {
        statement.targets.push_back("texmode_unified");
    }
    else
    {
        assertM(false, "Ocelot internal error - invalid token.");
    }
}

void PTXParserState::target()
{
    report( "  Rule: TARGET targetElementList" );
    statement.directive = ir::PTXStatement::Target;
}

void PTXParserState::noAddressSpace()
{
    statement.instruction.addressSpace = ir::PTXInstruction::Generic;
}

void PTXParserState::addressSpace( int value )
{
    statement.instruction.addressSpace = tokenToAddressSpace( value );
}

void PTXParserState::dataType( int value )
{
    operand.type = tokenToDataType( value );
}

void PTXParserState::instructionVectorType( int value )
{
    statement.instruction.vec = tokenToVec( value );
}

void PTXParserState::statementVectorType( int value )
{
    statement.array.vec = tokenToVec( value );
}

void PTXParserState::attribute( bool visible, bool external, bool weak )
{
    assert( visible + external + weak < 2 );
    if( visible )
    {
        statement.attribute = ir::PTXStatement::Visible;
    }
    else if( external )
    {
        statement.attribute = ir::PTXStatement::Extern;
    }
    else if( weak )
    {
        statement.attribute = ir::PTXStatement::Weak;
    }
    else
    {
        statement.attribute = ir::PTXStatement::NoAttribute;
    }
}

void PTXParserState::shiftAmount( bool shift )
{
    statement.instruction.shiftAmount = shift;
}

void PTXParserState::vectorIndex( int token )
{
    operand.vIndex = tokenToVectorIndex( token );
}

void PTXParserState::arrayDimensionSet( long long int value,
    YYLTYPE& location, bool add )
{
    report( "  Rule: '[' DECIMAL_CONSTANT ']'" );

    if( !add )
    {
        statement.array.stride.clear();
    }

    if( value <= 0 )
    {
        throw_exception( toString( location, *this )
            << "Invalid array dimension " << value, InvalidArray );
    }

    report( "   Got dimension " << value );
    statement.array.stride.push_back( ( unsigned int ) value );
}

void PTXParserState::arrayDimensionSet()
{
    report( "  Rule: arrayDimensions '[' ']'" );
    statement.array.stride.clear();

    report( "   Got dimension " << 0 );
    statement.array.stride.push_back( 0 );
}

void PTXParserState::arrayDimensions()
{
    statement.array.stride.clear();
}

void PTXParserState::statementEnd( YYLTYPE& location )
{
    statement.line   = location.first_line;
    statement.column = location.first_column;

    report( "   At (" << statement.line << "," << statement.column
        << ") : Parsed statement " << statements.size()
        << " \"" << statement.toString() << "\"" );
    statements.push_back( statement );

    operand = ir::PTXOperand();

    /*
    //
    // There is no reason why the PTXStatement constructor shouldn't 'reset' the
    // statement object. If there is, it should be made explicit.
    //
    statement.array.values.clear();
    statement.symbols.clear();
    alignment = 1;
    statement.array.vec = ir::PTXOperand::v1;
    */
    statement = ir::PTXStatement();
    statement.instruction.statementIndex = statements.size();
}

void PTXParserState::assignment()
{
    report( "  Clearing doubles" );
    statement.array.values.clear();
    statement.array.symbols.clear();
}

void PTXParserState::registerDeclaration( const std::string& name,
    YYLTYPE& location, unsigned int regs )
{
    report( "  Rule: REG dataType IDENTIFIER ';' : " << name
        << " ["  << regs << "]" );
    statement.directive = ir::PTXStatement::Reg;
    statement.type = operand.type;
    statement.name = name;
    statement.attribute = ir::PTXStatement::NoAttribute;

    if( operand.type == ir::PTXOperand::pred )
    {
        operand.condition = ir::PTXOperand::Pred;
    }

    statement.array.stride.resize(1);
    statement.array.stride[0] = regs;

    operand.vec = statement.array.vec;
    operand.addressMode = ir::PTXOperand::Register;

    if( regs == 0 )
    {
        statement.array.stride[0] = 0;

        if( _getOperandInScope( statement.name ) != 0 )
        {
            throw_exception( toString( location, *this )
                << "Variable name " << statement.name
                << " already declared in this scope.",
                DuplicateDeclaration );
        }

        if( operand.vec != ir::PTXOperand::v1 )
        {
            operand.array.push_back( ir::PTXOperand(
                ir::PTXOperand::Register, operand.type,
                statement.name + ".x" ) );
            operand.array.push_back( ir::PTXOperand(
                ir::PTXOperand::Register, operand.type,
                statement.name + ".y" ) );
            if( operand.vec != ir::PTXOperand::v2 ) {
                operand.array.push_back( ir::PTXOperand(
                    ir::PTXOperand::Register, operand.type,
                    statement.name + ".z" ) );
                operand.array.push_back( ir::PTXOperand(
                    ir::PTXOperand::Register, operand.type,
                    statement.name + ".w" ) );
            }
        }

        operand.identifier = statement.name;

        contexts.back().operands.insert( std::make_pair( statement.name,
            OperandWrapper( operand,
            statement.instruction.addressSpace ) ) );

        operand.array.clear();
    }

    for( unsigned int i = 0; i < regs; ++i )
    {
        std::stringstream name;
        name << statement.name << i;

        if( _getOperandInScope( name.str() ) != 0 )
        {
            throw_exception( toString( location, *this )
                << "Variable name " << statement.name
                << " already declared in this scope.",
                DuplicateDeclaration );
        }

        operand.identifier = name.str();

        if( operand.vec != ir::PTXOperand::v1 )
        {
            operand.array.push_back( ir::PTXOperand(
                ir::PTXOperand::Register, operand.type,
                name.str() + ".x" ) );
            operand.array.push_back( ir::PTXOperand(
                ir::PTXOperand::Register, operand.type,
                name.str() + ".y" ) );
            if( operand.vec != ir::PTXOperand::v2 ) {
                operand.array.push_back( ir::PTXOperand(
                    ir::PTXOperand::Register, operand.type,
                    name.str() + ".z" ) );
                operand.array.push_back( ir::PTXOperand(
                    ir::PTXOperand::Register, operand.type,
                    name.str() + ".w" ) );
            }
        }

        contexts.back().operands.insert( std::make_pair( name.str(),
            OperandWrapper( operand,
            statement.instruction.addressSpace ) ) );

        operand.array.clear();
    }
}

void PTXParserState::registerSeperator( YYLTYPE& location )
{
    ir::PTXOperand::DataType type = statement.type;
    statementEnd( location );
    operand.type = type;
}

void PTXParserState::initializableDeclaration( const std::string& name,
    YYLTYPE& one, YYLTYPE& two )
{
    report( "  Rule: initializable addressableVariablePrefix IDENTIFIER "
        << "arrayDimensions initializer ';'" );

    assert( directive == ir::PTXStatement::Const
        || directive == ir::PTXStatement::Global
        || directive == ir::PTXStatement::Shared
        || directive == ir::PTXStatement::Local);

    statement.directive = directive;
    report( "   Name = " << name );
    statement.name = name;
    statement.alignment = alignment;
    statement.type = operand.type;

    if( statement.array.values.size() != 0 )
    {
        unsigned int expected = 0;

        for( ir::PTXStatement::ArrayStrideVector::iterator
            fi = statement.array.stride.begin();
            fi != statement.array.stride.end(); ++fi )
        {
            expected += *fi;
        }

        if( statement.array.vec == ir::PTXOperand::v2 )
        {
            expected *= 2;
        }
        else if( statement.array.vec == ir::PTXOperand::v4 )
        {
            expected *= 4;
        }

        if( expected != statement.array.values.size() )
        {
            throw_exception( toString( two, *this )
                << "Array size " << expected
                << " does not match initializer size "
                << statement.array.values.size(),
                InitializerSizeMismatch );
        }
    }

    if( _getOperandInScope( statement.name ) != 0 )
    {
        throw_exception( toString( one, *this )
            << "Variable name " << statement.name
            << " already declared in this scope.",
            DuplicateDeclaration );
    }

    operand.identifier = statement.name;
    operand.addressMode = ir::PTXOperand::Address;
    contexts.back().operands.insert( std::make_pair( statement.name,
        OperandWrapper( operand, _toAddressSpace( directive ) ) ) );
}

void PTXParserState::textureDeclaration( int space,
    const std::string& name, YYLTYPE& location )
{
    report( "  Rule: textureSpace TOKEN_TEXREF identifier ';'" );

    statement.directive = ir::PTXStatement::Texref;
    statement.space = tokenToTextureSpace( space );
    statement.name = name;

    if( _getOperandInScope( statement.name ) != 0 )
    {
        throw_exception( toString( location, *this )
            << "Texture reference name " << statement.name
            << " already declared in this scope.",
            DuplicateDeclaration );
    }

    operand.identifier = statement.name;
    operand.addressMode = ir::PTXOperand::Address;
    contexts.back().operands.insert( std::make_pair( statement.name,
        OperandWrapper( operand, _toAddressSpace( directive ) ) ) );
}

void PTXParserState::surfaceDeclaration( int space,
    const std::string &name, YYLTYPE &location)
{
    report( "  Rule: textureSpace TOKEN_SURFREF identifier ';'" );

    statement.directive = ir::PTXStatement::Surfref;
    statement.space = tokenToTextureSpace( space );
    statement.name = name;

    if( _getOperandInScope( statement.name ) != 0 )
    {
        throw_exception( toString( location, *this )
            << "Texture reference name " << statement.name
            << " already declared in this scope.",
            DuplicateDeclaration );
    }

    operand.identifier = statement.name;
    operand.addressMode = ir::PTXOperand::Address;
    contexts.back().operands.insert( std::make_pair( statement.name,
        OperandWrapper( operand, _toAddressSpace( directive ) ) ) );
}

void PTXParserState::samplerDeclaration( int space,
    const std::string &name, YYLTYPE &location)
{
    report( "  Rule: textureSpace TOKEN_SURFREF identifier ';'" );

    statement.directive = ir::PTXStatement::Surfref;
    statement.space = tokenToTextureSpace( space );
    statement.name = name;

    if( _getOperandInScope( statement.name ) != 0 )
    {
        throw_exception( toString( location, *this )
            << "Texture reference name " << statement.name
            << " already declared in this scope.",
            DuplicateDeclaration );
    }

    operand.identifier = statement.name;
    operand.addressMode = ir::PTXOperand::Address;
    contexts.back().operands.insert( std::make_pair( statement.name,
        OperandWrapper( operand, _toAddressSpace( directive ) ) ) );
}

void PTXParserState::argumentDeclaration( const std::string& name,
    YYLTYPE& location )
{
    report( "  Rule: argument addressableVariablePrefix identifier "
        << " arrayDimensions" );

    assert( inEntry || inArgumentList || inReturnList );

    statement.directive = ir::PTXStatement::Param;
    statement.name = name;
    statement.alignment = alignment;
    statement.type = operand.type;

    operand.identifier = statement.name;
    operand.addressMode = ir::PTXOperand::Address;

    if( _getOperandInScope( statement.name ) != 0 )
    {
        throw_exception( toString( location, *this )
            << "Argument operand " << statement.name
            << " already declared in this scope.",
            DuplicateDeclaration );
    }

    contexts.back().operands.insert( std::make_pair( statement.name,
        OperandWrapper( operand, ir::PTXInstruction::Param ) ) );

    if( inReturnList )
    {
        prototype.returnTypes.push_back( operand.type );
    }
    else if( inArgumentList )
    {
        prototype.argumentTypes.push_back( operand.type );
    }

    statementEnd( location );
}

void PTXParserState::paramArgumentDeclaration(int token)
{
    report(" Rule: parameterAttribute: TOKEN_PTR kernelParameterPtrSpace");
    statement.ptrAddressSpace = tokenToAddressSpace(token);
    report("    address space: " << ir::PTXInstruction::toString(
        statement.ptrAddressSpace));
}

void PTXParserState::fileDeclaration( unsigned int file,
    const std::string& name )
{
    report( "  Rule: TOKEN_FILE DECIMAL_CONSTANT TOKEN_STRING: .file "
        << file << " " << name );
    statement.directive = ir::PTXStatement::File;
    statement.name = name;
    statement.sourceFile = file;
}

void PTXParserState::openBrace( YYLTYPE& location )
{
    report( "  Rule: '{'" );

    if( !inEntry )
    {
        inEntry = true;
    }

    statement.directive = ir::PTXStatement::StartScope;
    statementEnd( location );

    contexts.push_back( Context( contextId++ ) );
    report("Nesting level " << contexts.size());
}

void PTXParserState::closeBrace( YYLTYPE& location )
{
    report( "  Rule: '}'" );

    assert( inEntry );
    assert( contexts.size() > 1 );

    statement.directive = ir::PTXStatement::EndScope;

    // Set metadata for all instructions
    if( !statement.instruction.metadata.empty() )
    {
        unsigned int i = 0;
        for( ir::Module::StatementVector::reverse_iterator
            s = statements.rbegin(); s != statements.rend() &&
            i < contexts.back().instructionCount; ++s )
        {
            if( s->directive == ir::PTXStatement::Instr )
            {
                s->instruction.metadata = statement.instruction.metadata;
                ++i;
            }
        }
    }

    statementEnd( location );

    contexts.pop_back();
    report("Nesting level " << contexts.size());

    inEntry = contexts.size() > 1;
}

void PTXParserState::argumentListBegin( YYLTYPE& location )
{
    report( "  Rule: '('" );
    assert( !inArgumentList );
    inArgumentList = true;

    statement.directive = ir::PTXStatement::StartParam;
    statement.isReturnArgument = false;

    statementEnd( location );
}

void PTXParserState::argumentListEnd( YYLTYPE& location )
{
    report( "  Rule: ')'" );
    assert( inArgumentList );
    inArgumentList = false;

    statement.directive = ir::PTXStatement::EndParam;
    statement.isReturnArgument = false;

    statementEnd( location );
}

void PTXParserState::returnArgumentListBegin( YYLTYPE& location )
{
    report( "  Rule: '('" );
    assert( !inReturnList );
    inReturnList = true;

    statement.directive = ir::PTXStatement::StartParam;
    statement.isReturnArgument = true;

    statementEnd( location );
}

void PTXParserState::returnArgumentListEnd( YYLTYPE& location )
{
    report( "  Rule: ')'" );
    assert( inReturnList );
    inReturnList = false;

    statement.directive = ir::PTXStatement::EndParam;
    statement.isReturnArgument = false;

    statementEnd( location );
}

void PTXParserState::functionBegin( YYLTYPE& location )
{
    report( "  Rule: .func" );
    statement.directive = ir::PTXStatement::Func;

    contexts.push_back( Context( contextId++ ) );
    report("Nesting level " << contexts.size());

    statementEnd( location );
}

void PTXParserState::functionName( const std::string& name,
    YYLTYPE& location )
{
    report( "  Rule functionName : identifier" );

    statement.directive = ir::PTXStatement::FunctionName;
    statement.name = name;
    prototype.name = name;

    statementEnd( location );
}

void PTXParserState::functionDeclaration( YYLTYPE& location, bool body )
{
    report( "  Rule: externOrVisible functionBegin " <<
        "optionalReturnArgumentList identifier argumentList" );

    if( !body )
    {
        if( contexts.back().prototypes.count( prototype.name ) != 0 )
        {
            throw_exception( toString( location, *this )
                << "Function/Kernel " << prototype.name
                << " already declared in this scope.",
                DuplicateDeclaration );
        }

        assert( contexts.size() > 1 );

        contexts.pop_back();
        report("Nesting level " << contexts.size());

        statement.directive = ir::PTXStatement::EndFuncDec;
        statementEnd( location );
    }

    PrototypeMap::iterator proto =
        contexts.back().prototypes.find( prototype.name );
    if( proto != contexts.back().prototypes.end() )
    {
        if( !proto->second.compare( prototype ) )
        {
            throw_exception( toString( location, *this )
                << "Function/Kernel " << prototype.name
                << " already declared in this scope with "
                << "different arguments.\nHere: "
                << prototype.toString() << "\nPrevious:"
                << proto->second.toString(),
                DuplicateDeclaration );
        }
    }
    else
    {
        contexts.back().prototypes.insert( std::make_pair(
            prototype.name, prototype ) );
        contexts.back().operands.insert( std::make_pair( prototype.name,
            ir::PTXOperand( ir::PTXOperand::FunctionName,
            ir::PTXOperand::TypeSpecifier_invalid, prototype.name ) ) );
    }

    prototype.clear();
}

void PTXParserState::entry( const std::string& name, YYLTYPE& location )
{
    report( "  Rule: ENTRY identifier" );

    statement.directive = ir::PTXStatement::Entry;
    statement.name = name;

    prototype.name = name;

    contexts.push_back( Context( contextId++ ) );
    report("Nesting level " << contexts.size());

    statementEnd( location );
}

void PTXParserState::entryPrototype( YYLTYPE& location )
{
    report( "  Rule: entryPrototype" );

    assert( contexts.size() > 1 );

    statement.directive = ir::PTXStatement::StartScope;
    statementEnd( location );

    statement.directive = ir::PTXStatement::EndScope;
    statementEnd( location );

    contexts.pop_back();
    report("Nesting level " << contexts.size());

    inEntry = contexts.size() > 1;
}

void PTXParserState::entryDeclaration( YYLTYPE& location )
{
    report( "  Rule: entryName argumentList performanceDirectives" );

    if( contexts.back().prototypes.count( statement.name ) != 0 )
    {
        throw_exception( toString( location, *this )
            << "Function/Kernel " << statement.name
            << " already declared in this scope.",
            DuplicateDeclaration );
    }

    contexts.back().prototypes.insert(
        std::make_pair( prototype.name, prototype ) );
    contexts.back().operands.insert( std::make_pair( prototype.name,
        ir::PTXOperand( ir::PTXOperand::FunctionName,
        ir::PTXOperand::TypeSpecifier_invalid, prototype.name ) ) );

    prototype.clear();
}

void PTXParserState::entryStatement( YYLTYPE& location )
{
    statementEnd( location );

    report( "  Rule: guard instruction : "
        << statements.back().instruction.toString() );

    // check for an error
    assert( !statements.empty() );
    assert( statements.back().directive == ir::PTXStatement::Instr );

    std::string message = statements.back().instruction.valid();

    if( message != "" )
    {
        throw_exception( toString( location, *this )
            << "Parsed invalid instruction "
            << statements.back().instruction.toString()
            << " : " << message, InvalidInstruction );
    }

    operandVector.clear();
}

void PTXParserState::metadata( const std::string& metadata )
{
    report( "   Added metadata " << metadata);
    statement.instruction.metadata = metadata;
}

void PTXParserState::locationAddress( int token )
{
    directive = tokenToDirective( token );
    operand.addressMode = ir::PTXOperand::Address;
    operand.offset = 0;
}

void PTXParserState::uninitializableDeclaration( const std::string& name )
{
    report( "  Rule: uninitializable addressableVariablePrefix IDENTIFIER "
        << "arrayDimensions';' : " << name );

    assert( directive == ir::PTXStatement::Param ||
        directive == ir::PTXStatement::Local ||
        directive == ir::PTXStatement::Shared );

    statement.directive = directive;
    statement.name      = _nameInContext(name);
    statement.alignment = alignment;
    statement.type      = operand.type;

    operand.identifier  = statement.name;
    operand.addressMode = ir::PTXOperand::Address;

    contexts.back().operands.insert( std::make_pair(
        name, OperandWrapper( operand,
        _toAddressSpace( directive ) ) ) );
}

void PTXParserState::location( long long int one, long long int two,
    long long int three )
{
    report( "  Rule: LOC DECIMAL_CONSTANT DECIMAL_CONSTANT "
        << "DECIMAL_CONSTANT : " << one << ", " << two << ", " << three );

    std::stringstream stream;

    stream << one << " " << two << " " << three;

    statement.directive = ir::PTXStatement::Loc;
    statement.sourceFile = (unsigned int) one;
    statement.sourceLine = (unsigned int) two;
    statement.sourceColumn = (unsigned int) three;

    statement.name = stream.str();
}

void PTXParserState::label( const std::string& string )
{
    report( "  Rule: LABEL : " << string );

    statement.directive = ir::PTXStatement::Label;
    statement.name = string;
}

void PTXParserState::pragma( const std::string& string )
{
    report( "  Rule: PRAGMA : " << string );

    statement.directive = ir::PTXStatement::Pragma;
    statement.name = string;
}

void PTXParserState::labelOperand( const std::string& string )
{
    OperandWrapper* mode = _getOperand( string );

    if( mode == 0 )
    {
        operand.identifier = string;
        operand.addressMode = ir::PTXOperand::Label;
        operand.type = ir::PTXOperand::TypeSpecifier_invalid;

        operandVector.push_back( operand );
    }
    else
    {
        if( mode->operand.addressMode == ir::PTXOperand::Address )
        {
            statement.instruction.addressSpace = mode->space;
        }

        operandVector.push_back( *mode );
    }
}

void PTXParserState::nonLabelOperand( const std::string& string,
    YYLTYPE& location, bool invert )
{
    OperandWrapper* mode = _getOperand( string );

    if( mode == 0 )
    {
        throw_exception( toString( location, *this ) << "Operand "
            << string << " not declared in this scope.", NoDeclaration );
    }

    if( mode->operand.addressMode == ir::PTXOperand::Address )
    {
        statement.instruction.addressSpace = mode->space;
        operand = mode->operand;
    }
    else if( mode->operand.addressMode == ir::PTXOperand::Register
        || mode->operand.addressMode == ir::PTXOperand::Special )
    {
        if( mode->operand.vec != ir::PTXOperand::v1 )
        {
            switch( operand.vIndex )
            {
                case ir::PTXOperand::ix:
                {
                    operand = mode->operand.array[ 0 ];
                    break;
                }
                case ir::PTXOperand::iy:
                {
                    operand = mode->operand.array[ 1 ];
                    break;
                }
                case ir::PTXOperand::iz:
                {
                    operand = mode->operand.array[ 2 ];
                    break;
                }
                case ir::PTXOperand::iw:
                {
                    operand = mode->operand.array[ 3 ];
                    break;
                }
                default:
                {
                    operand = mode->operand;
                    break;
                }
            }
        }
        else
        {
            operand = mode->operand;
        }
    }
    else
    {
        operand = mode->operand;
    }

    if( invert )
    {
        if( operand.type != ir::PTXOperand::pred )
        {
            throw_exception( toString( location, *this ) << "Operand "
                << string << " is not a predicate and can't be inverted.",
                NotPredicate );
        }
        operand.condition = ir::PTXOperand::InvPred;
    }

    operandVector.push_back( OperandWrapper( operand, mode->space ) );
}

void PTXParserState::constantOperand( long long int value )
{
    operand.addressMode = ir::PTXOperand::Immediate;
    operand.imm_int = value;
    operand.vec = ir::PTXOperand::v1;
    operand.type = smallestType( operand.imm_int );
    operandVector.push_back( operand );
}

void PTXParserState::constantOperand( unsigned long long int value )
{
    operand.addressMode = ir::PTXOperand::Immediate;
    operand.imm_int = value;
    operand.vec = ir::PTXOperand::v1;
    operand.type = smallestType( operand.imm_uint );
    operandVector.push_back( operand );
}

void PTXParserState::constantOperand( float value )
{
    report("SINGLE_CONSTANT: " << value);
    operand.addressMode = ir::PTXOperand::Immediate;
    operand.imm_single = value;
    operand.vec = ir::PTXOperand::v1;
    operand.type = ir::PTXOperand::f32;
    operandVector.push_back( operand );
}

void PTXParserState::constantOperand( double value )
{
    report("DOUBLE_CONSTANT: " << value);
    operand.addressMode = ir::PTXOperand::Immediate;
    operand.imm_float = value;
    operand.vec = ir::PTXOperand::v1;
    operand.type = ir::PTXOperand::f64;
    operandVector.push_back( operand );
}

void PTXParserState::indexedOperand( const std::string& name,
    YYLTYPE& location, long long int index )
{
    OperandWrapper* mode = _getOperand( name );

    if( mode == 0 )
    {
        throw_exception( toString( location, *this ) << "Operand "
            << name << " not declared in this scope.", NoDeclaration );
    }

    operand.addressMode = ir::PTXOperand::Address;

    operand.identifier = name;
    operand.vec = ir::PTXOperand::v1;
    operand.array.clear();

    operand.offset = (int) index * ir::PTXOperand::bytes(
        mode->operand.type);
    operand.type = mode->operand.type;

    operandVector.push_back( OperandWrapper( operand, mode->space ) );
}


void PTXParserState::addressableOperand( const std::string& name,
    long long int value, YYLTYPE& location, bool invert )
{
    OperandWrapper* mode = _getOperand( name );

    if( mode == 0 )
    {
        throw_exception( toString( location, *this ) << "Operand "
            << name << " not declared in this scope.", NoDeclaration );
    }

    if( mode->operand.addressMode == ir::PTXOperand::Register )
    {
        operand.addressMode = ir::PTXOperand::Indirect;
    }
    else
    {
        report(" Operand: " << name << " mode: "
            << ir::PTXOperand::toString(
            mode->operand.addressMode));
        operand.addressMode = mode->operand.addressMode;
    }

    operand.identifier = mode->operand.identifier;
    operand.vec = ir::PTXOperand::v1;
    operand.array.clear();

    if( invert )
    {
        value = -value;
    }

    operand.offset = (int) value;
    operand.type = mode->operand.type;

    operandVector.push_back( OperandWrapper( operand, mode->space ) );
}

void PTXParserState::arrayOperand( YYLTYPE& location )
{
    assert( !identifiers.empty() );

    report("  Rule: arrayOperand()");

    OperandWrapper* mode = _getOperand( identifiers.front() );

    if( identifiers.size() > 4 )
    {
        throw_exception( toString( location, *this )
            << "Array operand \""
            << hydrazine::toString( identifiers.begin(),
            identifiers.end(), "," )
            << "\" has more than 4 elements.", InvalidArray );
    }

    if( mode == 0 )
    {
        throw_exception( toString( location, *this ) << "Operand "
            << identifiers.front() << " not declared in this scope.",
            NoDeclaration );
    }

    operand.addressMode = ir::PTXOperand::Register;
    operand.type        = mode->operand.type;

    if( identifiers.size() == 1 )
    {
        operand.vec = ir::PTXOperand::v1;

        operand.array.push_back( mode->operand );

        report("    pushing operand " << mode->operand.toString());
    }
    else if( identifiers.size() >= 2 )
    {
        operand.vec = ir::PTXOperand::v2;
        operand.array.push_back( mode->operand );

        mode = _getOperand( identifiers[1] );

        if( mode == 0 )
        {
            throw_exception( toString( location, *this ) << "Operand "
                << identifiers[1] << " not declared in this scope.",
                NoDeclaration );
        }

        operand.array.push_back( mode->operand );
    }

    if( identifiers.size() == 4 )
    {

        operand.vec = ir::PTXOperand::v4;

        mode = _getOperand( identifiers[2] );

        if( mode == 0 )
        {
            throw_exception( toString( location, *this ) << "Operand "
                << identifiers[2] << " not declared in this scope.",
                NoDeclaration );
        }

        operand.array.push_back( mode->operand );

        mode = _getOperand( identifiers[3] );

        if( mode == 0 )
        {
            throw_exception( toString( location, *this ) << "Operand "
                << identifiers[3] << " not declared in this scope.",
                NoDeclaration );
        }

        operand.array.push_back( mode->operand );
    }

    operandVector.push_back( operand );
    operand.array.clear();
}

void PTXParserState::returnOperand()
{
    ++returnOperands;
}

void PTXParserState::guard( const std::string& name, YYLTYPE& location,
    bool invert )
{
    report( "  Rule: PREDICATE_IDENTIFIER : " << name  );

    OperandWrapper* mode = _getOperand( name );

    if( mode == 0 )
    {
        throw_exception( toString( location, *this ) << "Predciate "
            << name << " not declared in this scope.", NoDeclaration );
    }

    operand = mode->operand;
    if( invert )
    {
        operand.condition = ir::PTXOperand::InvPred;
    }
    else
    {
        operand.condition = ir::PTXOperand::Pred;
    }

    if( mode->operand.addressMode == ir::PTXOperand::Address )
    {
        statement.instruction.addressSpace = mode->space;
    }

    operandVector.push_back( operand );
}

void PTXParserState::guard()
{
    report( "  Rule: No guard predicate" );
    operand.addressMode = ir::PTXOperand::Register;
    operand.type = ir::PTXOperand::pred;
    operand.condition = ir::PTXOperand::PT;
    operandVector.push_back( operand );
}

void PTXParserState::tail( bool condition )
{
    statement.instruction.tailCall = condition;
}

void PTXParserState::uni( bool condition )
{
    statement.instruction.uni = condition;
}

void PTXParserState::carry( bool condition )
{
    if( condition )
    {
        report( "  Rule: Carry" );
        statement.instruction.carry = ir::PTXInstruction::CC;
        statement.instruction.pq.type = ir::PTXOperand::u32;
        statement.instruction.pq.addressMode = ir::PTXOperand::Register;
        statement.instruction.pq.vec = ir::PTXOperand::v1;
        statement.instruction.pq.identifier = "%_ZconditionCode";
    }
    else
    {
        report( "  Rule: No Carry" );
        statement.instruction.carry = ir::PTXInstruction::None;
    }
}

void PTXParserState::full()
{
    statement.instruction.divideFull = true;
}

void PTXParserState::modifier( int token )
{
    statement.instruction.modifier |= tokenToModifier( token );
}

void PTXParserState::atomic( int token )
{
    statement.instruction.atomicOperation = tokenToAtomicOperation( token );
}

void PTXParserState::volatileFlag( bool condition )
{
    if( condition )
    {
        statement.instruction.volatility = ir::PTXInstruction::Volatile;
    }
    else
    {
        statement.instruction.volatility = ir::PTXInstruction::Nonvolatile;
    }
}

void PTXParserState::reduction( int token )
{
    statement.instruction.reductionOperation
        = tokenToReductionOperation( token );
}

void PTXParserState::comparison( int token )
{
    statement.instruction.comparisonOperator = tokenToCmpOp( token );
}

void PTXParserState::boolean( int token )
{
    statement.instruction.booleanOperator = tokenToBoolOp( token );
}

void PTXParserState::geometry( int token )
{
    statement.instruction.geometry = tokenToGeometry( token );
}

void PTXParserState::vote( int token )
{
    statement.instruction.vote = tokenToVoteMode( token );
}

void PTXParserState::shuffle( int token )
{
    statement.instruction.shuffleMode = tokenToShuffleMode( token );
}

void PTXParserState::level( int token )
{
    statement.instruction.level = tokenToLevel( token );
}

void PTXParserState::permute( int token )
{
    statement.instruction.permuteMode = tokenToPermuteMode( token );
}

void PTXParserState::floatingPointMode( int token )
{
    statement.instruction.floatingPointMode
        = tokenToFloatingPointMode( token );
}

void PTXParserState::defaultPermute()
{
    statement.instruction.permuteMode = ir::PTXInstruction::DefaultPermute;
}

void PTXParserState::instruction()
{
    statement.instruction = ir::PTXInstruction( );
    statement.instruction.statementIndex = statements.size();
    contexts.back().instructionCount++;
}

void PTXParserState::instruction( const std::string& opcode,
    int dataType )
{
    statement.directive = ir::PTXStatement::Instr;
    statement.instruction.type = tokenToDataType( dataType );
    statement.instruction.opcode = stringToOpcode( opcode );
    statement.instruction.pg = operandVector[0].operand;

    unsigned int index = 1;

    if( operandVector.size() > index )
    {
        statement.instruction.d = operandVector[index++].operand;
    }

    if( operandVector.size() > index )
    {
        if( ( operandVector[ index ].operand.type == ir::PTXOperand::pred
            && operandVector.size() > 4 ) || operandVector.size() == 6 )
        {
            statement.instruction.pq = operandVector[index++].operand;
        }
    }

    if( operandVector.size() > index )
    {
        statement.instruction.a = operandVector[index++].operand;
    }
    if( operandVector.size() > index )
    {
        statement.instruction.b = operandVector[index++].operand;
    }
    if( operandVector.size() > index )
    {
        statement.instruction.c = operandVector[index++].operand;
    }

    _setImmediateTypes();
}

void PTXParserState::instruction( const std::string& opcode )
{
    instruction( opcode, TOKEN_B64 );
}

void PTXParserState::tex( int dataType )
{
    report( "  Rule: instruction : tex" );

    assert( operandVector.size() == 4 );

    statement.directive          = ir::PTXStatement::Instr;
    statement.instruction.type   = tokenToDataType( dataType );
    statement.instruction.opcode = stringToOpcode( "tex" );
    statement.instruction.pg     = operandVector[0].operand;
    statement.instruction.d      = operandVector[1].operand;
    statement.instruction.a      = operandVector[2].operand;
    statement.instruction.c      = operandVector[3].operand;

    _setImmediateTypes();
}

void PTXParserState::tld4( int dataType )
{
    report( "  Rule: instruction : tld4" );

    assert( operandVector.size() == 4 );

    statement.directive          = ir::PTXStatement::Instr;
    statement.instruction.type   = tokenToDataType( dataType );
    statement.instruction.opcode = stringToOpcode( "tld4" );
    statement.instruction.pg     = operandVector[0].operand;
    statement.instruction.d      = operandVector[1].operand;
    statement.instruction.a      = operandVector[2].operand;
    statement.instruction.c      = operandVector[3].operand;

    _setImmediateTypes();
}

void PTXParserState::callPrototypeName( const std::string& identifier )
{
    report( "  Rule: callPrototypeName '" << identifier << "'" );
    prototype.name = identifier;
}

void PTXParserState::call( const std::string& identifier,
    YYLTYPE& location )
{
    report( "  Rule: instruction : call" );

    statement.directive = ir::PTXStatement::Instr;
    statement.instruction.opcode = stringToOpcode( "call" );

    statement.instruction.pg = operandVector[0].operand;

    OperandWrapper* operand = _getOperand( identifier );

    if( operand == 0 )
    {
        throw_exception( toString( location, *this )
            << "Function call name/operand '"
            << identifier << "' not declared in this scope.",
            InvalidInstruction );
    }

    report( "   name: " << identifier );

    statement.instruction.a = operand->operand;

    if( statement.instruction.a.addressMode == ir::PTXOperand::Register )
    {
        statement.instruction.c.addressMode = ir::PTXOperand::FunctionName;
        statement.instruction.c.identifier  = prototype.name;
    }
    else if( prototype.name.empty() )
    {
        prototype.name = statement.instruction.a.identifier;
    }

    FunctionPrototype* pi = _getPrototype( prototype.name );
    if( pi == 0 )
    {
        throw_exception( toString( location, *this )
            << "Function/Prototype '"
            << prototype.name
            << "' not declared in this scope.",
            NoDeclaration );
    }

    statement.instruction.b.addressMode = ir::PTXOperand::ArgumentList;

    FunctionPrototype proto;

    report( "   return operands(" << returnOperands << "):" );

    if( returnOperands > 0 )
    {
        statement.instruction.d.addressMode = ir::PTXOperand::ArgumentList;
    }

    unsigned int operandIndex = 1;
    for( ; returnOperands > 0; --returnOperands, ++operandIndex )
    {
        ir::PTXOperand& operand = operandVector[ operandIndex ].operand;

        report( "    " << operand.toString() );

        if( operand.addressMode == ir::PTXOperand::BitBucket )
        {
            operand.type = pi->returnTypes[	proto.returnTypes.size() ];
        }

        if( operand.addressMode == ir::PTXOperand::Address )
        {
            auto addressSpace = operandVector[ operandIndex ].space;

            if( addressSpace != ir::PTXInstruction::Param )
            {
                throw_exception( toString( location, *this )
                    << "Function call return argument '"
                    << operand.identifier
                    << "' is NOT in the parameter address space.",
                    InvalidInstruction );
            }
        }

        proto.returnTypes.push_back( operand.type );

        statement.instruction.d.array.push_back(
            operandVector[ operandIndex ].operand );
    }

    report( "   operands:" );

    for( ; operandIndex < operandVector.size(); ++operandIndex )
    {
        ir::PTXOperand& operand = operandVector[ operandIndex ].operand;

        report( "    " << operand.toString() );

        if( operand.addressMode == ir::PTXOperand::BitBucket )
        {
            operand.type = pi->argumentTypes[
                proto.argumentTypes.size() ];
        }

        if( operand.addressMode == ir::PTXOperand::Address )
        {
            auto addressSpace = operandVector[ operandIndex ].space;

            if( addressSpace != ir::PTXInstruction::Param )
            {
                throw_exception( toString( location, *this )
                    << "Function call argument '"
                    << operand.identifier
                    << "' is NOT in the parameter address space.",
                    InvalidInstruction );
            }
        }

        proto.argumentTypes.push_back( operand.type );

        statement.instruction.b.array.push_back(
            operandVector[ operandIndex ].operand );
    }

    if( !pi->compare( proto ) )
    {
        throw_exception( toString( location, *this )
            << " Call instruction '" << statement.instruction.toString()
            << "' does not match prototype '"
            << pi->toString() << "'.", PrototypeMismatch );
    }

    prototype.clear();
}

void PTXParserState::carryIn()
{
    ir::PTXOperand conditionCode( ir::PTXOperand::Register,
        ir::PTXOperand::u32 );
    conditionCode.identifier = "%_ZconditionCode";
    operandVector.push_back( conditionCode );
}

void PTXParserState::relaxedConvert( int token, YYLTYPE& location )
{
    if( !ir::PTXOperand::relaxedValid( tokenToDataType( token ),
        statement.instruction.a.type )
        && statement.instruction.a.addressMode == ir::PTXOperand::Register )
    {
        throw_exception( toString( location, *this )
            << "Type of " << statement.instruction.a.toString() << " "
            << ir::PTXOperand::toString( statement.instruction.a.type )
            << " not convertable to type "
            << ir::PTXOperand::toString( tokenToDataType( token ) )
            << " using relaxed conversion rules.", InvalidDataType );
    }

    statement.instruction.a.relaxedType = tokenToDataType( token );
}

void PTXParserState::cvtaTo()
{
    statement.instruction.toAddrSpace = true;
}

void PTXParserState::convert( int token, YYLTYPE& location )
{
    if( !ir::PTXOperand::valid( tokenToDataType( token ),
        statement.instruction.a.type ) )
    {
        throw_exception( toString( location, *this )
            << "Type of " << statement.instruction.a.identifier << " "
            << ir::PTXOperand::toString( statement.instruction.a.type )
            << " not convertable to type "
            << ir::PTXOperand::toString( tokenToDataType( token ) )
            << " .", InvalidDataType );
    }

    statement.instruction.a.type = tokenToDataType( token );
}

void PTXParserState::convertC( int token, YYLTYPE& location )
{
    if( !ir::PTXOperand::valid( tokenToDataType( token ),
        statement.instruction.c.type ) )
    {
        throw_exception( toString( location, *this )
            << "Type of " << statement.instruction.c.identifier << " "
            << ir::PTXOperand::toString( statement.instruction.c.type )
            << " not convertable to type "
            << ir::PTXOperand::toString( tokenToDataType( token ) )
            << " .", InvalidDataType );
    }

    statement.instruction.c.type = tokenToDataType( token );
}

void PTXParserState::convertD( int token, YYLTYPE& location )
{
    if( !ir::PTXOperand::valid( tokenToDataType( token ),
        statement.instruction.d.type ) )
    {
        throw_exception( toString( location, *this )
            << "Type of " << statement.instruction.d.identifier << " "
            << ir::PTXOperand::toString( statement.instruction.d.type )
            << " not convertable to type "
            << ir::PTXOperand::toString( tokenToDataType( token ) )
            << " .", InvalidDataType );
    }

    statement.instruction.d.type = tokenToDataType( token );
}

void PTXParserState::operandCIsAPredicate()
{
    statement.instruction.c.type = ir::PTXOperand::pred;
}
void PTXParserState::cacheOperation(int token) {
    statement.instruction.cacheOperation = tokenToCacheOperation(token);
}

void PTXParserState::cacheLevel(int token ) {
    statement.instruction.cacheLevel = tokenToCacheLevel(token);
}

void PTXParserState::clampOperation(int token) {
    statement.instruction.clamp = tokenToClampOperation(token);
}

void PTXParserState::barrierOperation( int token, YYLTYPE & location) {
    statement.instruction.barrierOperation = tokenToBarrierOp(token);
}

void PTXParserState::formatMode(int token) {
    statement.instruction.formatMode = tokenToFormatMode(token);
}

void PTXParserState::surfaceQuery(int token) {
    report("surfaceQuery(" << token << ")");
    statement.instruction.surfaceQuery = tokenToSurfaceQuery(token);
}

void PTXParserState::colorComponent(int token) {
    statement.instruction.colorComponent = tokenToColorComponent(token);
}

void PTXParserState::returnType( int token )
{
    report( "  Rule returnType: dataTypeId identifier" );

    prototype.returnTypes.push_back( tokenToDataType( token ) );
}

void PTXParserState::argumentType( int token )
{
    report( "  Rule argumentType: dataTypeId identifier" );

    prototype.argumentTypes.push_back( tokenToDataType( token ) );
}

void PTXParserState::callPrototype( const std::string& name,
    const std::string& identifier, YYLTYPE& location )
{
    report( "  Rule callPrototype: TOKEN_LABEL TOKEN_CALL_PROTOTYPE "
        << "returnTypeList identifier argumentTypeList ';'" );

    prototype.name = name;

    PrototypeMap::iterator duplicate =
        contexts.back().prototypes.find( name );

    if( duplicate != contexts.back().prototypes.end() )
    {
        if( !prototype.compare( duplicate->second ) )
        {
            throw_exception( toString( location, *this )
                << "Function/Kernel prototype " << name
                << " already declared in this scope with "
                "different arguments.",
                DuplicateDeclaration );
        }
    }

    report( "   name: '" << name << "'" );

    contexts.back().prototypes.insert( std::make_pair( name, prototype ) );

    if( identifier != "_" )
    {
        contexts.back().operands.insert( std::make_pair( identifier,
            ir::PTXOperand( ir::PTXOperand::FunctionName,
            ir::PTXOperand::TypeSpecifier_invalid, identifier ) ) );
    }

    statement.directive     = ir::PTXStatement::FunctionPrototype;
    statement.returnTypes   = prototype.returnTypes;
    statement.argumentTypes = prototype.argumentTypes;
    statement.name          = prototype.name;

    prototype.clear();
}

void PTXParserState::callTargets( const std::string& name,
    YYLTYPE& location )
{
    report( "  Rule: TOKEN_LABEL TOKEN_CALL_TARGETS stringList ';'" );
    statement.directive = ir::PTXStatement::CallTargets;
    statement.targets.assign( identifiers.begin(), identifiers.end() );

    // check the targets to make sure that they exist
    for( StringList::iterator target = identifiers.begin();
        target != identifiers.end(); ++target )
    {
        if( _getPrototype( *target ) == 0 )
        {
            throw_exception( toString( location, *this )
                << "Function named '" << *target
                << "' not declared in this scope.", NoDeclaration );
        }
    }

    identifiers.clear();
}

} // namespace parser
