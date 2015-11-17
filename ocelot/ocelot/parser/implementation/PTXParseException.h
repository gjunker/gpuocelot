#ifndef PTXPARSEEXCEPTION_H
#define PTXPARSEEXCEPTION_H

#include <exception>
#include <string>

namespace parser {

    class PTXParseException : public std::exception
    {
        public:

            enum Error
            {
                Success,
                SyntaxError,
                MalformedVersion,
                InvalidDataType,
                InvalidVecType,
                InitializerSizeMismatch,
                InvalidInstruction,
                DuplicateDeclaration,
                NoDeclaration,
                InvalidOpcode,
                DuplicateLabel,
                NoPrototype,
                PrototypeMismatch,
                NoLabel,
                InvalidArray,
                NotPredicate,
                NotSupported,
                NotVersion2_1,
                Invalid
            };

            std::string message;
            Error error;

        public:
            const char* what() const throw();
            ~PTXParseException() throw();
    };

} // namespace parser

#endif // PTXPARSEEXCEPTION_H
