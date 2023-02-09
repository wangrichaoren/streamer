set(CALC_SOURCE_FILES
        main.cpp
        MathOperationDataModel.cpp
        NumberDisplayDataModel.cpp
        NumberSourceDataModel.cpp
        ImageLoaderModel.cpp
        CvImageLoaderModel.cpp
        ImageShowModel.cpp
        CvImageShowModel.cpp
        Utils.cpp
        CvRGB2GrayModel.cpp
        CvBinaryModel.cpp
        )

set(CALC_HEADER_FILES
        AdditionModel.hpp
        DivisionModel.hpp
        DecimalData.hpp
        MathOperationDataModel.hpp
        NumberDisplayDataModel.hpp
        NumberSourceDataModel.hpp
        SubtractionModel.hpp
        ImageData.hpp
        ImageLoaderModel.hpp
        CvImageLoaderModel.hpp
        ImageShowModel.hpp
        CvImageShowModel.hpp
        StringData.hpp
        Utils.hpp
        CvRGB2GrayModel.hpp
        CvBinaryModel.hpp
        )

add_executable(calculator
        ${CALC_SOURCE_FILES}
        ${CALC_HEAEDR_FILES}
        )

target_link_libraries(calculator QtNodes)


set(HEADLESS_CALC_SOURCE_FILES
        headless_main.cpp
        MathOperationDataModel.cpp
        NumberDisplayDataModel.cpp
        NumberSourceDataModel.cpp
        )

add_executable(headless_calculator
        ${HEADLESS_CALC_SOURCE_FILES}
        ${CALC_HEAEDR_FILES}
        )

target_link_libraries(headless_calculator QtNodes)
