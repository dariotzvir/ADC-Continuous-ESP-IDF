//
// Created by dario on 12/7/22.
//

#ifndef ADC_CONTINUOUS_ADC_CONTINUOUS_H
#define ADC_CONTINUOUS_ADC_CONTINUOUS_H

//STD C++:
#include <array>
#include <utility>
//FreeRTOS:
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//ESP-IDF:
#include <esp_log.h>
#include <hal/adc_types.h>
#include <esp_adc/adc_continuous.h>
#include <soc/soc_caps.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

namespace ESPIDF_Cxx
{
    #if CONFIG_IDF_TARGET_ESP32
    constexpr adc_digi_convert_mode_t  ADC_CONV_MODE   = ADC_CONV_SINGLE_UNIT_1;  //ESP32 only supports ADC1 DMA mode
    constexpr adc_digi_output_format_t ADC_OUTPUT_TYPE = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
    #elif CONFIG_IDF_TARGET_ESP32S2
    constexpr adc_digi_convert_mode_t  ADC_CONV_MODE   = ADC_CONV_BOTH_UNIT
    constexpr adc_digi_output_format_t ADC_OUTPUT_TYPE = ADC_DIGI_OUTPUT_FORMAT_TYPE2
    #elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32H4 || CONFIG_IDF_TARGET_ESP32C2
    constexpr adc_digi_convert_mode_t  ADC_CONV_MODE   = ADC_CONV_ALTER_UNIT     //ESP32C3 only supports alter mode
    constexpr adc_digi_output_format_t ADC_OUTPUT_TYPE = ADC_DIGI_OUTPUT_FORMAT_TYPE2
    #elif CONFIG_IDF_TARGET_ESP32S3
    constexpr adc_digi_convert_mode_t  ADC_CONV_MODE   = ADC_CONV_BOTH_UNIT
    constexpr adc_digi_output_format_t ADC_OUTPUT_TYPE = ADC_DIGI_OUTPUT_FORMAT_TYPE2
    #endif

    using config_canal = adc_digi_pattern_config_t;
    using frame_config = adc_continuous_handle_cfg_t;

    template<size_t size>
    using array_canales  = std::array<config_canal, size>;

    template<size_t size>
    using array_lecturas = std::array<float, size>;

    bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                         const adc_continuous_evt_data_t *edata,
                                         void *user_data);
    bool IRAM_ATTR on_pool_ov(adc_continuous_handle_t handle,
                                     const adc_continuous_evt_data_t *edata,
                                     void *user_data);

    template<size_t size>
    class ADC_continuous
    {
        const size_t n_size = size;
        const array_canales<size> canales;
        const frame_config frame;
        const uint32_t sample_frec;
        bool asd = false;

        adc_continuous_config_t adc_config;
        adc_continuous_handle_t handle;
        adc_cali_handle_t calib_handle;
        bool esta_calibrado;


        inline std::pair<bool, size_t> buscar_indice_canal(uint16_t canal);
        inline std::pair<bool, size_t> buscar_indice_canal(uint16_t canal,
                                                             uint16_t unidad);

    public:
        static TaskHandle_t task_handle;

        ADC_continuous(const array_canales<size> &canales_arg,
                       const uint32_t &sample_frec_arg,
                       frame_config frame_arg);
        //frame_config se pasa por copia ya que daba un error de:
        //template variable sized object may not be initialized

        void init();
        void deinit();
        void esperar_lectura();
        void calibrar(adc_cali_handle_t &calib);
        void cancelar_calibrar();
        void cambiar_handle(TaskHandle_t task_handle_arg);
        std::pair<esp_err_t, array_lecturas<size>> leer_prom();
    };

    template<size_t size>
    TaskHandle_t ADC_continuous<size>::task_handle;


    template<size_t size>
    inline std::pair<bool, size_t> ADC_continuous<size>::buscar_indice_canal
    (uint16_t canal)
    {
        for(size_t i=0; i<size; i++)
            if(canal == canales[i].channel) return {true, i};
        return {false, 0};
    }

    template<size_t size>
    inline std::pair<bool, size_t> ADC_continuous<size>::buscar_indice_canal
    (uint16_t canal, [[maybe_unused]] uint16_t unidad)
    {
        for(size_t i=0; i<size; i++)
            if(canal == canales[i].channel) return {true, i};

        return {false, 0};
    }

    template<size_t size>
    ADC_continuous<size>::ADC_continuous(const array_canales<size> &canales_arg,
                                         const uint32_t &sample_frec_arg,
                                         frame_config frame_arg):
                   canales(canales_arg), frame(frame_arg),
                   sample_frec(sample_frec_arg)
    {
        cambiar_handle(xTaskGetCurrentTaskHandle());

        esta_calibrado = false;
        adc_config = (adc_continuous_config_t)
        {
                .pattern_num     = size,
                .adc_pattern     = (adc_digi_pattern_config_t*) canales.data(),
                .sample_freq_hz  = sample_frec,
                .conv_mode       = ADC_CONV_MODE,
                .format          = ADC_OUTPUT_TYPE,
        };
    }

    template<size_t size>
    void ADC_continuous<size>::init()
    {
        ESP_ERROR_CHECK(adc_continuous_new_handle(&frame , &handle));
        ESP_ERROR_CHECK(adc_continuous_config(handle, &adc_config));
        adc_continuous_evt_cbs_t cbs =
        {
            .on_conv_done = s_conv_done_cb,
            .on_pool_ovf  = on_pool_ov,
        };
        ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, (void *)&n_size));
        ESP_ERROR_CHECK(adc_continuous_start(handle));
    }

    template<size_t size>
    void ADC_continuous<size>::deinit()
    {
        ESP_ERROR_CHECK(adc_continuous_stop(handle));
        ESP_ERROR_CHECK(adc_continuous_deinit(handle));
    }

    template<size_t size>
    void ADC_continuous<size>::esperar_lectura()
    {
        constexpr TickType_t espera = millis_a_ticks(1000);
        uint32_t r = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /*if(r != 1)
        {
            LOG(LIGHT_CYAN, "Se vencio tiempo de espera");
            //auto r = adc_continuous_stop(handle);
            deinit();
            asd = true;
            LOG(LIGHT_CYAN, "ADC parado");
            vTaskDelay(espera);
            //r = adc_continuous_start(handle);
            init();
            LOG(LIGHT_CYAN, "ADC reiniciado");
        }*/
    }

    template<size_t size>
    void ADC_continuous<size>::calibrar(adc_cali_handle_t &calib)
    {
        calib_handle = calib;
        esta_calibrado = true;
    }
    template<size_t size>
    void ADC_continuous<size>::cancelar_calibrar()
    {
        esta_calibrado = false;
    }
    template<size_t size>
    std::pair<esp_err_t, array_lecturas<size>> ADC_continuous<size>::leer_prom()
    {
        array_lecturas<size>       lecturas;
        std::array<uint64_t, size> sum;
        std::array<uint64_t, size> cont;
        sum.fill(0);
        cont.fill(0);

        uint8_t raw[frame.conv_frame_size];
        memset(raw, 0xcc, frame.conv_frame_size);
        uint32_t muestras_hechas = 0;
        auto err = adc_continuous_read(handle,
                                       raw, frame.conv_frame_size,
                                       &muestras_hechas, 0);
        if(err != ESP_OK) return {err, lecturas};

        for(uint32_t i = 0; i < muestras_hechas; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            auto *p = (adc_digi_output_data_t *) (&raw[i]);

            uint16_t adc_raw = p->type1.data;
            auto [encontrado, index] = buscar_indice_canal(p->type1.channel);
            if(!encontrado) continue;

            if(esta_calibrado)
            {
                int tension;
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(calib_handle, adc_raw, &tension));

                LOG(DARK_GREEN, "Tension %u: %f", index, tension);
                sum[index] += tension;
            }
            else sum[index] += adc_raw;

            cont[index]++;
        }

        for(size_t i = 0; i < size; i++)
        {
            if(cont[i] != 0 ) lecturas[i] = float(sum[i]) / cont[i];
            if(asd) LOG(LIGHT_CYAN, "lec %u: %f", i, lecturas[i]);
        }

        return {err, lecturas};
    }

    template<size_t size>
    void ADC_continuous<size>::cambiar_handle(TaskHandle_t task_handle_arg)
    {
        ADC_continuous<size>::task_handle = task_handle_arg;
    }

    bool s_conv_done_cb(
            [[maybe_unused]] adc_continuous_handle_t handle,
            [[maybe_unused]] const adc_continuous_evt_data_t *edata,
            [[maybe_unused]] void *user_data)
    {
        auto &task_handle = ESPIDF_Cxx::ADC_continuous<6>::task_handle;
        BaseType_t desperto_hilo = pdFALSE;
        if(task_handle != nullptr)
            xTaskNotifyFromISR(task_handle, 1, eSetValueWithOverwrite, &desperto_hilo);

        return (desperto_hilo == pdTRUE);
    }

    bool on_pool_ov(
            [[maybe_unused]] adc_continuous_handle_t handle,
            [[maybe_unused]] const adc_continuous_evt_data_t *edata,
            [[maybe_unused]] void *user_data)
    {
        return pdTRUE;
    }
}

#endif //ADC_CONTINUOUS_ADC_CONTINUOUS_H
