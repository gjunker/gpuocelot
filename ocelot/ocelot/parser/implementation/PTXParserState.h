#ifndef PTXPARSERSTATE_H
#define PTXPARSERSTATE_H

#include <ocelot/ir/interface/Module.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct YYLTYPE;

namespace parser {

    class PTXParserState
    {
        public:
            class OperandWrapper
            {
                public:
                    OperandWrapper( const ir::PTXOperand& o,
                        ir::PTXInstruction::AddressSpace s
                        = ir::PTXInstruction::AddressSpace_Invalid );
                public:
                    ir::PTXOperand operand;
                    ir::PTXInstruction::AddressSpace space;
            };

            class FunctionPrototype
            {
                public:
                    typedef std::vector< ir::PTXOperand::DataType >
                        TypeVector;

                public:
                    TypeVector returnTypes;
                    TypeVector argumentTypes;
                    std::string name;

                public:
                    void clear();
                    bool compare( const FunctionPrototype& t );
                    std::string toString() const;
            };

            typedef std::unordered_map< std::string,
                OperandWrapper > OperandMap;
            typedef std::unordered_map< std::string,
                FunctionPrototype > PrototypeMap;

            class Context
            {
            public:
                Context( unsigned int id );

            public:
                unsigned int id;
                OperandMap   operands;
                PrototypeMap prototypes;
                unsigned int instructionCount;
            };

            typedef std::unordered_map< std::string, unsigned int >
                StringMap;
            typedef std::vector< std::string > StringList;
            typedef std::vector< OperandWrapper > OperandVector;
            typedef std::vector< Context > ContextStack;

        public:
            ir::Module::StatementVector statements;
            StringMap labels;
            std::string fileName;

        public:
            std::string sectionType;
            std::string sectionName;

            StringList identifiers;

            ContextStack contexts;
            unsigned int contextId;

            bool inEntry;
            bool inArgumentList;
            bool inReturnList;

            unsigned int returnOperands;
            unsigned int alignment;
            ir::PTXOperand operand;
            OperandVector operandVector;
            ir::PTXStatement statement;
            FunctionPrototype prototype;

            ir::PTXStatement::Directive directive;
            std::string comment;


        private:
            static ir::PTXInstruction::AddressSpace _toAddressSpace(
                ir::PTXStatement::Directive directive );

        private:
            void _setImmediateTypes();
            std::string _nameInContext( const std::string& name );

            OperandWrapper* _getOperand( const std::string& name );
            OperandWrapper* _getOperandInScope(
                const std::string& name );
            FunctionPrototype* _getPrototype( const std::string& name );

        public:
            void addSpecialRegisters();

            void maxnreg( unsigned int regs );
            void maxntid( unsigned int tidx, unsigned int tidy = 1024,
                unsigned int tidz = 1024 );
            void ctapersm( int target, unsigned int ctas );
            void maxnctapersm( unsigned int ctas );
            void maxnctapersm();
            void minnctapersm( unsigned int ctas );
            void minnctapersm();

        public:
            void preprocessor( int token );
            void version( double version, YYLTYPE& location );
            void addressSize( unsigned int size );
            void identifierList( const std::string& identifier );
            void identifierList2( const std::string& identifier );
            void decimalListSingle( long long int value );
            void decimalListSingle2( long long int value );
            void symbolListSingle( const std::string& identifier );
            void symbolListSingle2( const std::string& identifier );
            void floatList( double value );
            void floatList1( double value );
            void singleList( float value );
            void singleList1( float value );
            void targetElement( int token );
            void target();
            void noAddressSpace();
            void addressSpace( int token );
            void dataType( int token );
            void statementVectorType( int token );
            void instructionVectorType( int token );
            void attribute( bool visible, bool external, bool weak );
            void shiftAmount( bool shift );
            void vectorIndex( int token );

            void arrayDimensionSet( long long int value,
                YYLTYPE& location, bool add );
            void arrayDimensionSet();
            void arrayDimensions();
            void assignment();
            void registerDeclaration( const std::string& name,
                YYLTYPE& location, unsigned int regs = 0 );
            void registerSeperator( YYLTYPE& location );
            void fileDeclaration( unsigned int id,
                const std::string& name );
            void initializableDeclaration( const std::string& name,
                YYLTYPE& one, YYLTYPE& two );
            void textureDeclaration( int token,const std::string& name,
                YYLTYPE& location );
            void surfaceDeclaration( int token,
                const std::string &name, YYLTYPE &location);
            void samplerDeclaration( int token,
                const std::string &name, YYLTYPE &location);
            void argumentDeclaration( const std::string& name,
                YYLTYPE& location );
            void paramArgumentDeclaration(int token);

            void openBrace( YYLTYPE& location );
            void closeBrace( YYLTYPE& location );
            void returnArgumentListBegin( YYLTYPE& location );
            void returnArgumentListEnd( YYLTYPE& location );
            void argumentListBegin( YYLTYPE& location );
            void argumentListEnd( YYLTYPE& location );

            void functionBegin( YYLTYPE& location );
            void functionName( const std::string& name,
                YYLTYPE& location );
            void functionDeclaration( YYLTYPE& location, bool body );

            void entry( const std::string& name, YYLTYPE& location );
            void entryDeclaration( YYLTYPE& location );
            void entryPrototype( YYLTYPE& location );
            void entryStatement( YYLTYPE& location );
            void metadata( const std::string& comment );

            void locationAddress( int token );
            void uninitializableDeclaration( const std::string& name );
            void location( long long int one, long long int two,
                long long int three );
            void label( const std::string& string );
            void pragma( const std::string& string );
            void labelOperand( const std::string& string );
            void nonLabelOperand( const std::string& string,
                YYLTYPE& location, bool invert );
            void constantOperand( long long int value );
            void constantOperand( unsigned long long int value );
            void constantOperand( float value );
            void constantOperand( double value );
            void indexedOperand( const std::string& name,
                YYLTYPE& location, long long int value );
            void addressableOperand( const std::string& name,
                long long int value, YYLTYPE& location,
                bool invert );
            void arrayOperand( YYLTYPE& location );
            void returnOperand();
            void guard( const std::string& name, YYLTYPE& one,
                bool invert );
            void guard();
            void statementEnd( YYLTYPE& location );

            void tail( bool condition );
            void uni( bool condition );
            void carry( bool condition );
            void modifier( int token );
            void atomic( int token );
            void volatileFlag( bool condition );
            void reduction( int token );
            void comparison( int token );
            void boolean( int token );
            void geometry( int token );
            void vote( int token );
            void shuffle( int token );
            void level( int token );
            void permute( int token );
            void floatingPointMode( int token );
            void defaultPermute();
            void full();

            void instruction();
            void instruction( const std::string& opcode, int dataType );
            void instruction( const std::string& opcode );
            void tex( int dataType );
            void tld4( int dataType );
            void callPrototypeName( const std::string& identifier );
            void call( const std::string& identifier,
                YYLTYPE& location );
            void carryIn();
            void relaxedConvert( int token, YYLTYPE& location );
            void cvtaTo();
            void convert( int token, YYLTYPE& location );
            void convertC( int token, YYLTYPE& location );
            void convertD( int token, YYLTYPE& location );
            void operandCIsAPredicate();
            void barrierOperation( int token, YYLTYPE & location);
            void cacheOperation(int token );
            void cacheLevel(int token );
            void clampOperation(int token);
            void formatMode(int token);
            void surfaceQuery(int token);
            void colorComponent(int token);

            void returnType( int token );
            void argumentType( int token );
            void callPrototype( const std::string& name,
                const std::string& identifier, YYLTYPE& location );
            void callTargets( const std::string& name,
                YYLTYPE& location );
    };

    std::string
    toString( YYLTYPE& location, parser::PTXParserState& state );

} // namespace parser

#endif // PTXPARSERSTATE_H
