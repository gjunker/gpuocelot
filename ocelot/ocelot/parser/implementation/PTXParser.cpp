/*! \file PTXParser.cpp
	\date Monday January 19, 2009
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\brief The source file for the PTXParser class.
*/

#ifndef PTX_PARSER_CPP_INCLUDED
#define PTX_PARSER_CPP_INCLUDED

// Ocelot Includes
#include "PTXParser.h"
#include "PTXParseException.h"
#include <ptxgrammar.hpp>

// Hydrazine Includes
#include <hydrazine/interface/debug.h>
#include <hydrazine/interface/string.h>

// Standard Library Includes
#include <cassert>

#ifdef REPORT_BASE
#undef REPORT_BASE
#endif

#define REPORT_BASE 0

/*! \brief A namespace for parsing PTX */
namespace ptx
{ 
    extern int yyparse( parser::PTXLexer&, parser::PTXParserState& );
}

namespace parser
{
	void PTXParser::checkLabels()
	{
		typedef std::unordered_map< std::string, ir::PTXStatement > 
			StatementMap;
		
		report( "Checking labels." );
		
		StatementMap labels;
		
		ir::Module::StatementVector::iterator 
			begin = state.statements.begin();
		
		unsigned int depth = 0;
		
		for( ir::Module::StatementVector::iterator 
			statement = state.statements.begin(); 
			statement != state.statements.end(); ++statement )
		{
			if( statement->directive == ir::PTXStatement::Label )
			{
				report( " Found label " << statement->name );
			
				StatementMap::iterator label = 
					labels.find( statement->name );
				
				if( label != labels.end() )
				{
					std::stringstream error;
					error << state.fileName << " ("  << statement->line << "," 
						<< statement->column << "): Duplicate label "
						<< statement->name << " previously defined at (" 
						<< label->second.line << "," << label->second.column 
						<< ")";
						
                    PTXParseException exception;
					exception.message = error.str();
                    exception.error = PTXParseException::DuplicateLabel;
					throw exception;
				}
				
				labels.insert( std::make_pair( statement->name, *statement ) );
			}
			else if( statement->directive == ir::PTXStatement::StartScope )
			{
				++depth;
			}
			else if( statement->directive == ir::PTXStatement::EndScope )
			{
				if( depth-- == 1 )
				{
					for( ir::Module::StatementVector::iterator 
						s = begin; s != statement; ++s )
					{
						if( s->directive == ir::PTXStatement::Instr )
						{
							ir::PTXOperand* operands[] = { &s->instruction.a, 
								&s->instruction.b, &s->instruction.c, 
								&s->instruction.d };
				
							for( unsigned int i = 0; i < 4; ++i )
							{
								ir::PTXOperand& operand = *operands[ i ];
				
								if( operand.addressMode == ir::PTXOperand::Label )
								{
									StatementMap::iterator label = 
										labels.find( operand.identifier );
				
									if( label == labels.end() )
									{
										std::stringstream error;
										error << state.fileName << " ("  
											<< s->line << "," << s->column 
											<< "): Label "
											<< operand.identifier 
											<< " not delcared in this scope";
						
                                        PTXParseException exception;
										exception.message = error.str();
                                        exception.error = PTXParseException::NoLabel;
										throw exception;
									}
								}
							}
						}
					}
				
					labels.clear();
					begin = statement;
				}
			}
		}	
	}
	
	void PTXParser::reset()
	{
		report("PTXParser::reset() called");
		state.inEntry = false;
		state.inReturnList = false;
		state.inArgumentList = false;
		state.identifiers.clear();

		state.contexts.clear();
		state.contextId = 1;
		
        state.contexts.push_back( PTXParserState::Context( 0 ) );
		state.operandVector.clear();
		report("Reset nesting level " << state.contexts.size());		

		state.returnOperands = 0;

		state.statements.clear();
		state.fileName = fileName;
		
		ir::PTXOperand bucket;
		bucket.identifier = "_";
		bucket.type = ir::PTXOperand::b64;
		bucket.addressMode = ir::PTXOperand::BitBucket;
		bucket.vec = ir::PTXOperand::v1;
		
		state.contexts.back().operands.insert(
			std::make_pair( bucket.identifier, 
            PTXParserState::OperandWrapper( bucket,
			ir::PTXInstruction::Global ) ) );
		
		ir::PTXOperand pt;
		pt.identifier = "%pt";
		pt.type = ir::PTXOperand::pred;
		pt.addressMode = ir::PTXOperand::Register;
		pt.condition = ir::PTXOperand::PT;
		pt.vec = ir::PTXOperand::v1;
		
		state.contexts.back().operands.insert( std::make_pair( pt.identifier, 
            PTXParserState::OperandWrapper( pt,
			ir::PTXInstruction::Global ) ) );
		
		state.addSpecialRegisters();	
	}

	std::string PTXParser::getLinesNearCurrentLocation( std::istream& input )
	{
		if( !input.good() )
		{
			input.clear();
		}

		input.seekg( 0, std::ios::beg );
		
		unsigned int beginLine = 0;
		
		if(!state.statements.empty())
		{
			beginLine = state.statements.back().line;
		}
		
		const unsigned int offset = 10;

		if(beginLine > offset) beginLine -= offset;
		
		unsigned int endLine = beginLine + 2 * offset;
		
		unsigned int currentLine = 0;
		
		while(input.good())
		{
			if(currentLine == beginLine) break;

			char n = input.get();
			
			if(n == '\n')
			{
				++currentLine;
			}
		}
		
		std::string result;
		
		while(input.good())
		{
			if(currentLine >= endLine) break;

			char n = input.get();
			
			result += n;
			
			if(n == '\n')
			{
				++currentLine;
			}
		}
		
		return hydrazine::addLineNumbers(result, beginLine + 1);
	}
	
#if 0
	ir::PTXOperand::DataType PTXParser::tokenToDataType( int token )
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

	ir::PTXOperand::Vec PTXParser::tokenToVec( int token )
	{
		switch( token )
		{
			case TOKEN_V2: return ir::PTXOperand::v2; break;
			case TOKEN_V4: return ir::PTXOperand::v4; break;			
			default:
			{
                PTXParseException exception;
                exception.error = PTXParseException::InvalidVecType;
				
				std::stringstream stream;
				stream << token;
				exception.message = "Got invalid vector type " + stream.str();
				throw exception;
				break;
			}
		}
		
		return ir::PTXOperand::v1;
	}

	ir::PTXOperand::VectorIndex PTXParser::tokenToVectorIndex( int token )
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

	ir::PTXInstruction::Opcode PTXParser::stringToOpcode( std::string string )
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

	
	ir::PTXInstruction::Modifier PTXParser::tokenToModifier( int token )
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
	
	ir::PTXInstruction::AddressSpace PTXParser::tokenToAddressSpace( int token )
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

	ir::PTXStatement::Directive PTXParser::tokenToDirective( int token )
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
		PTXParser::tokenToReductionOperation( int token )
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
		PTXParser::tokenToAtomicOperation( int token )
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
	
	ir::PTXInstruction::CmpOp PTXParser::tokenToCmpOp( int token )
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
	
	ir::PTXInstruction::CacheOperation PTXParser::tokenToCacheOperation(int token)
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
	
	ir::PTXInstruction::CacheLevel PTXParser::tokenToCacheLevel(int token) {
		switch (token) {
			case TOKEN_L1: return ir::PTXInstruction::L1;
			case TOKEN_L2: return ir::PTXInstruction::L2;
			default: break;
		}
		return ir::PTXInstruction::CacheLevel_invalid;
	}
	
	ir::PTXInstruction::ClampOperation PTXParser::tokenToClampOperation(int token) {
		switch (token)
		{
			case TOKEN_TRAP: return ir::PTXInstruction::TrapOOB;
			case TOKEN_CLAMP: return ir::PTXInstruction::Clamp;
			case TOKEN_ZERO: return ir::PTXInstruction::Zero;
			default: break;
		}
		return ir::PTXInstruction::ClampOperation_Invalid;
	}
	
	ir::PTXInstruction::FormatMode PTXParser::tokenToFormatMode(int token)
	{
		switch (token)
		{
			case TOKEN_B: return ir::PTXInstruction::Unformatted;
			case TOKEN_P: return ir::PTXInstruction::Formatted;
			default: break;
		}
		return ir::PTXInstruction::FormatMode_Invalid;
	}
	
	ir::PTXInstruction::SurfaceQuery PTXParser::tokenToSurfaceQuery(int token)
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
	
	ir::PTXInstruction::ColorComponent PTXParser::tokenToColorComponent(
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

	ir::PTXInstruction::BarrierOperation PTXParser::tokenToBarrierOp(int token)
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
	
	ir::PTXInstruction::BoolOp PTXParser::tokenToBoolOp( int token )
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

	ir::PTXInstruction::Geometry PTXParser::tokenToGeometry( int token )
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
	
	ir::PTXInstruction::VoteMode PTXParser::tokenToVoteMode( int token )
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
	
	ir::PTXInstruction::ShuffleMode PTXParser::tokenToShuffleMode( int token )
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
	
	ir::PTXInstruction::Level PTXParser::tokenToLevel( int token )
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
	
	ir::PTXInstruction::PermuteMode PTXParser::tokenToPermuteMode( int token )
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

	ir::PTXInstruction::FloatingPointMode PTXParser::tokenToFloatingPointMode( 
		int token )
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

	ir::PTXStatement::TextureSpace PTXParser::tokenToTextureSpace( int token )
	{
		switch( token )
		{
			case TOKEN_GLOBAL: return ir::PTXStatement::GlobalSpace;    break;
			case TOKEN_PARAM:  return ir::PTXStatement::ParameterSpace; break;
		}
		return ir::PTXStatement::InvalidSpace;
	}
	
	ir::PTXOperand::DataType PTXParser::smallestType( long long int value )
	{
		return ir::PTXOperand::s64;
	}
	
	ir::PTXOperand::DataType 
		PTXParser::smallestType( long long unsigned int value )
	{
		return ir::PTXOperand::u64;
	}
#endif
	PTXParser::PTXParser()
	{

	}
				
	void PTXParser::parse( std::istream& input, 
		ir::Instruction::Architecture language )
	{
		assert( language == ir::Instruction::PTX );
	
		std::stringstream temp;
		
		report( "Parsing file " << fileName );
		report( "Running main parse pass." );
		
		parser::PTXLexer lexer( &input, &temp );
		reset();
		
		try 
		{
			state.addSpecialRegisters();
			ptx::yyparse( lexer, state );
			assertM( temp.str().empty(),
				"Failed to lex all characters, remainder is:\n"
				<< (int)temp.str()[0] );
		
			checkLabels();
		}
        catch( PTXParseException& e )
		{
			e.message = "\nFailed to parse file '" + fileName + "':\n" +
				getLinesNearCurrentLocation(input) +
				"\n" + e.message;
			
			report("parse error");
			report(e.what());
			throw e;
		}
	}
	
	ir::Module::StatementVector&& PTXParser::statements()
	{
		return std::move( state.statements );
	}
		
}

#endif

