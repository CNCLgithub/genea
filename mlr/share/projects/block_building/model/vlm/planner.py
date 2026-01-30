import os
import time

from dotenv import load_dotenv
from openai import OpenAI

from mlr.share.projects.block_building.utils.core_utils import ConfigUtils
from mlr.share.projects.block_building.utils.file_utils import FileUtils
from mlr.share.projects.block_building.utils.msg_utils import Msg
from mlr.share.projects.block_building.utils.vlm_utils import OpenAIUtils


class VLMPlanner:
    def __init__(self):
        pass

    def run_experiment(self):
        pass


class OpenAIPlanner(VLMPlanner):
    def __init__(self, exp_details_filepath, out_filepath, img_filepaths_list):
        super().__init__()

        load_dotenv()
        _openai_org = os.getenv('OPENAI_ORG_ID')
        _openai_api = os.getenv('OPENAI_API_KEY')
        self.openai_client = OpenAI(organization=_openai_org, api_key=_openai_api)

        self._exp_details = self._load_exp_details(exp_details_filepath)

        self._out_filepath = out_filepath
        FileUtils.create_out_file(self._out_filepath)

        self._img_filepaths_list = img_filepaths_list

    @staticmethod
    def _load_exp_details(json_filepath):
        return FileUtils.read_json_file(json_filepath)

    def _run_instructions(self, input_message):
        openai_message = []
        if input_message:
            openai_message = input_message[:]

        instructions_list = self._exp_details["instructions"]
        for instruction in instructions_list:
            instruction_type = instruction["type"]
            if instruction_type == "text":
                text_content = OpenAIUtils.get_text_content(instruction["prompt"])
                openai_message.append(OpenAIUtils.construct_message(dev_message=text_content))

        return openai_message

    def _run_practice(self, input_message):
        openai_message = []
        if input_message:
            openai_message = input_message[:]

        practice_list = self._exp_details["practice"]

        for practice in practice_list:
            practice_type = practice["type"]
            if practice_type == "text":
                content = OpenAIUtils.get_text_content(practice["prompt"])
                openai_message.append(OpenAIUtils.construct_message(usr_message=content))
            elif practice_type == "bool_t":
                content = OpenAIUtils.get_text_content(practice["prompt"])
                openai_message.append(OpenAIUtils.construct_message(usr_message=content))
                content = OpenAIUtils.get_text_content("True.")
                openai_message.append(OpenAIUtils.construct_message(asst_message=content))
            elif practice_type == "bool_f":
                content = OpenAIUtils.get_text_content(practice["prompt"])
                openai_message.append(OpenAIUtils.construct_message(usr_message=content))
                content = OpenAIUtils.get_text_content("False.")
                openai_message.append(OpenAIUtils.construct_message(asst_message=content))

        return openai_message

    def _run_openai_model(self, openai_message, max_tries=5):
        total_tries = 0
        while True:
            try:
                openai_response = self.openai_client.chat.completions.create(model=ConfigUtils.OPENAI_MODEL_NAME,
                                                                             messages=openai_message)
                break
            except Exception as e:
                total_tries += 1
                time.sleep(2 ** total_tries)
                if total_tries >= max_tries:
                    Msg.print_error("Max retry exhausted and an error still occurs persistently.")
                    raise e
        return openai_response

    def run_trial(self, input_message, input_img_filepaths):
        openai_message = []
        if input_message:
            openai_message = input_message[:]

        trial_list = self._exp_details["trial"]
        for trial, img_filepath in zip(trial_list, input_img_filepaths):
            trial_type = trial["type"]
            if trial_type == "text_img":
                content = OpenAIUtils.get_img_content(input_prompt=trial["prompt"], img_filename=img_filepath)
                openai_message.append(OpenAIUtils.construct_message(usr_message=content))

        FileUtils.write_as_json(openai_message)
        return self._run_openai_model(openai_message)

    def run_experiment(self):
        openai_message = self._run_instructions(None)
        openai_message = self._run_practice(openai_message)

        for img_filepaths in self._img_filepaths_list:
            openai_response = self.run_trial(openai_message, img_filepaths)
            self.record_responses(img_filepaths[0], openai_response.choices[0].message.content)
            trial_name = FileUtils.get_file_basename(img_filepaths[0], is_attached=False)[0]
            Msg.print_warn(f"Trial,{trial_name},Response,{openai_response.choices[0].message.content}")

    def record_responses(self, img_filepath, openai_response):
        trial_name = FileUtils.get_file_basename(img_filepath, is_attached=False)[0]
        FileUtils.write_row_to_file(self._out_filepath, [trial_name, openai_response])
